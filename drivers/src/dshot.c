/**
 * @file dshot.c
 * @brief Digital DShot & Bidirectional Telemetry (BDShot/EDT) Driver Implementation.
 * @organization Purdue ROV
 */

#include "dshot.h"

/* Universal 5B4B GCR Lookup Table */
static const uint8_t s_gcr_decode_table[32] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x09, 0x0A,
                                               0x0B, 0xFF, 0x0D, 0x0E, 0x0F, 0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05,
                                               0x06, 0x07, 0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF};

/* XOR of the three nibbles of a 12-bit payload; shared by command and telemetry frames. */
static uint16_t dshot_checksum(uint16_t payload_12) {
    return (uint16_t)((payload_12 ^ (payload_12 >> 4) ^ (payload_12 >> 8)) & 0x0FU);
}

uint16_t dshot_prepare_packet(uint16_t value, bool telemetry, bool invert_crc) {
    uint16_t payload = (uint16_t)(((value & DSHOT_VALUE_MASK) << 1) | (telemetry ? 1U : 0U));
    uint16_t csum = dshot_checksum(payload);
    if (invert_crc) {
        csum = (uint16_t)(~csum & 0x0FU);
    }
    return (uint16_t)((payload << 4) | csum);
}

bool dshot_decode_telemetry_frame(uint32_t raw_21_bits, uint16_t *out_telemetry_16) {
    if (out_telemetry_16 == (void *)0) {
        return false;
    }

    /* 1. Undo differential encoding: value ^ (value >> 1) */
    uint32_t gcr = raw_21_bits ^ (raw_21_bits >> 1);

    /* 2. Decode 4 nibbles using 5B4B GCR table */
    uint8_t n3 = s_gcr_decode_table[(gcr >> 15) & 0x1FU];
    uint8_t n2 = s_gcr_decode_table[(gcr >> 10) & 0x1FU];
    uint8_t n1 = s_gcr_decode_table[(gcr >> 5) & 0x1FU];
    uint8_t n0 = s_gcr_decode_table[gcr & 0x1FU];

    if (n3 == 0xFF || n2 == 0xFF || n1 == 0xFF || n0 == 0xFF) {
        return false;
    }

    uint16_t frame = (uint16_t)((n3 << 12) | (n2 << 8) | (n1 << 4) | n0);

    /* 3. Check 4-bit CRC (uninverted on telemetry back-channel) */
    if (dshot_checksum((uint16_t)(frame >> 4)) != (frame & 0x0FU)) {
        return false;
    }

    *out_telemetry_16 = frame;
    return true;
}

dshot_telemetry_type_t dshot_parse_telemetry(uint16_t frame_16, uint8_t motor_pole_pairs, dshot_telemetry_t *telem) {
    if (telem == (void *)0) {
        return DSHOT_TELEMETRY_NONE;
    }

    uint16_t data = frame_16 >> 4;

    /* EDT frames are the "impossible" eRPM encodings: bit8 == 0 with a
     * non-zero exponent. All normalized eRPM frames have bit8 == 1 or
     * exponent == 0, so that is the exact discriminator. */
    uint8_t exponent = (uint8_t)((data >> 9) & 0x07U);
    bool is_edt = ((data & 0x0100U) == 0U) && (exponent != 0U);

    if (is_edt) {
        /* EDT Frame: prefix = eee0 in the top nibble */
        uint8_t type = (uint8_t)((data >> 8) & 0x0EU);
        uint8_t val = (uint8_t)(data & 0xFFU);

        switch (type) {
        case 0x02: /* Temperature in C */
            telem->temperature_c = (int16_t)val;
            telem->last_type = DSHOT_TELEMETRY_TEMPERATURE;
            return DSHOT_TELEMETRY_TEMPERATURE;

        case 0x04: /* Voltage: 0.25V per step */
            telem->voltage_v = (float)val * 0.25f;
            telem->last_type = DSHOT_TELEMETRY_VOLTAGE;
            return DSHOT_TELEMETRY_VOLTAGE;

        case 0x06: /* Current in Amperes */
            telem->current_a = (float)val;
            telem->last_type = DSHOT_TELEMETRY_CURRENT;
            return DSHOT_TELEMETRY_CURRENT;

        case 0x0C: /* Stress level [0-255] */
            telem->stress_level = val;
            telem->last_type = DSHOT_TELEMETRY_STRESS;
            return DSHOT_TELEMETRY_STRESS;

        case 0x0E: /* Status: bit7 alert, bit6 warning, bit5 error, bits3-0 max stress */
            telem->status_flags = (uint8_t)(val & 0xE0U);
            telem->status_max_stress = (uint8_t)(val & 0x0FU);
            telem->last_type = DSHOT_TELEMETRY_STATUS;
            return DSHOT_TELEMETRY_STATUS;

        default:
            telem->last_type = DSHOT_TELEMETRY_DEBUG;
            return DSHOT_TELEMETRY_DEBUG;
        }
    } else {
        /* eRPM Frame */
        uint16_t mantissa = data & 0x01FFU;
        uint32_t period_us = (uint32_t)mantissa << exponent;

        if (period_us == 0U || period_us >= DSHOT_ERPM_PERIOD_STOPPED) {
            telem->erpm = 0U;
            telem->rpm = 0U;
        } else {
            telem->erpm = 60000000UL / period_us;
            telem->rpm = dshot_erpm_period_to_rpm(period_us, motor_pole_pairs);
        }

        telem->last_type = DSHOT_TELEMETRY_ERPM;
        return DSHOT_TELEMETRY_ERPM;
    }
}

void dshot_cmd_queue_init(dshot_cmd_queue_t *queue, bool invert_crc) {
    if (queue == (void *)0) {
        return;
    }
    queue->state = DSHOT_CMD_STATE_IDLE;
    queue->command_frame = 0;
    queue->repeat_count = 0;
    queue->invert_crc = invert_crc;
}

bool dshot_cmd_request(dshot_cmd_queue_t *queue, uint8_t cmd_code) {
    if (queue == (void *)0 || cmd_code == 0 || cmd_code >= 48) {
        return false;
    }
    /* Do not cut a command short: settings commands need every repeat to take effect. */
    if (queue->state == DSHOT_CMD_STATE_SENDING) {
        return false;
    }
    /* Command frame requires Telemetry bit = 1 */
    queue->command_frame = dshot_prepare_packet(cmd_code, true, queue->invert_crc);
    queue->repeat_count = DSHOT_CMD_REPEAT_COUNT;
    queue->state = DSHOT_CMD_STATE_SENDING;
    return true;
}

uint16_t dshot_cmd_get_frame(dshot_cmd_queue_t *queue, uint16_t throttle, bool telemetry) {
    if (queue == (void *)0) {
        return dshot_prepare_packet(throttle, telemetry, false);
    }
    if (queue->state == DSHOT_CMD_STATE_SENDING) {
        uint16_t frame = queue->command_frame;
        if (queue->repeat_count > 0) {
            queue->repeat_count--;
        }
        if (queue->repeat_count == 0) {
            queue->state = DSHOT_CMD_STATE_IDLE;
        }
        return frame;
    }
    return dshot_prepare_packet(throttle, telemetry, queue->invert_crc);
}

uint32_t dshot_erpm_period_to_rpm(uint32_t period_us, uint8_t pole_pairs) {
    if (period_us == 0 || pole_pairs == 0) {
        return 0;
    }
    return (uint32_t)(60000000ULL / ((uint64_t)period_us * (uint64_t)pole_pairs));
}
