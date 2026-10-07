/**
 * @file test_driver_dshot.c
 * @brief Unit tests for the DShot / Bidirectional DShot (BDShot) protocol driver.
 * @organization Purdue ROV
 */

#include "dshot.h"
#include <assert.h>
#include <stdio.h>

/* 4B5B GCR encode table (inverse of the driver's decode table). */
static const uint8_t k_gcr_encode[16] = {0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
                                         0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F};

/* Build a 16-bit telemetry word with a valid (uninverted) checksum. */
static uint16_t make_telemetry_word(uint16_t data12) {
    uint16_t csum = (uint16_t)((data12 ^ (data12 >> 4) ^ (data12 >> 8)) & 0x0FU);
    return (uint16_t)((data12 << 4) | csum);
}

/* Encode a 16-bit telemetry word into the 21-bit differential GCR stream an ESC sends. */
static uint32_t encode_telemetry_stream(uint16_t word) {
    uint32_t gcr = 0U;
    for (int nibble = 3; nibble >= 0; nibble--) {
        gcr = (gcr << 5) | k_gcr_encode[(word >> (nibble * 4)) & 0x0FU];
    }
    /* Invert gcr = raw ^ (raw >> 1), working from the MSB down. */
    uint32_t raw = 0U;
    for (int i = 20; i >= 0; i--) {
        uint32_t bit = ((gcr >> i) & 1U) ^ ((raw >> (i + 1)) & 1U);
        raw |= bit << i;
    }
    return raw;
}

static void test_prepare_packet(void) {
    /* Throttle 0, no telemetry: all zero, inverted CRC sets the low nibble. */
    assert(dshot_prepare_packet(0U, false, false) == 0x0000U);
    assert(dshot_prepare_packet(0U, false, true) == 0x000FU);

    /* Telemetry bit only. */
    assert(dshot_prepare_packet(0U, true, false) == 0x0011U);

    /* 1046: packet12 = 0x82C, csum = 0x82C ^ 0x82 ^ 0x8 = 0x6. */
    assert(dshot_prepare_packet(1046U, false, false) == 0x82C6U);
    assert(dshot_prepare_packet(1046U, false, true) == 0x82C9U);

    /* Out-of-range values must be masked to 11 bits, never leak into the telemetry bit. */
    assert(dshot_prepare_packet((uint16_t)(0x0800U | 1046U), false, false) == 0x82C6U);

    printf("[PASS] test_prepare_packet\n");
}

static void test_decode_telemetry_frame(void) {
    uint16_t out = 0U;
    uint16_t word = make_telemetry_word(0x22DU);

    assert(dshot_decode_telemetry_frame(encode_telemetry_stream(word), &out));
    assert(out == word);

    /* Corrupt checksum is rejected. */
    uint16_t bad_crc = (uint16_t)(word ^ 0x0001U);
    assert(!dshot_decode_telemetry_frame(encode_telemetry_stream(bad_crc), &out));

    /* Invalid GCR symbols (all-zero line) are rejected. */
    assert(!dshot_decode_telemetry_frame(0U, &out));

    assert(!dshot_decode_telemetry_frame(encode_telemetry_stream(word), NULL));

    printf("[PASS] test_decode_telemetry_frame\n");
}

static void test_parse_erpm(void) {
    dshot_telemetry_t t = {0};

    /* period = 500 << 1 = 1000 us -> 60000 eRPM -> 8571 RPM at 7 pole pairs. */
    uint16_t data = (uint16_t)((1U << 9) | 500U);
    assert(dshot_parse_telemetry(make_telemetry_word(data), 7U, &t) == DSHOT_TELEMETRY_ERPM);
    assert(t.erpm == 60000U);
    assert(t.rpm == 8571U);

    /* 0xFFF is the "motor stopped" sentinel and must read as zero. */
    assert(dshot_parse_telemetry(make_telemetry_word(0x0FFFU), 7U, &t) == DSHOT_TELEMETRY_ERPM);
    assert(t.erpm == 0U);
    assert(t.rpm == 0U);

    /* Unknown pole count must not leave a stale RPM behind. */
    t.rpm = 1234U;
    assert(dshot_parse_telemetry(make_telemetry_word(data), 0U, &t) == DSHOT_TELEMETRY_ERPM);
    assert(t.erpm == 60000U);
    assert(t.rpm == 0U);

    assert(dshot_parse_telemetry(make_telemetry_word(data), 7U, NULL) == DSHOT_TELEMETRY_NONE);

    printf("[PASS] test_parse_erpm\n");
}

static void test_parse_edt(void) {
    dshot_telemetry_t t = {0};

    assert(dshot_parse_telemetry(make_telemetry_word(0x22DU), 7U, &t) == DSHOT_TELEMETRY_TEMPERATURE);
    assert(t.temperature_c == 45);

    assert(dshot_parse_telemetry(make_telemetry_word(0x464U), 7U, &t) == DSHOT_TELEMETRY_VOLTAGE);
    assert(t.voltage_v > 24.99f && t.voltage_v < 25.01f);

    assert(dshot_parse_telemetry(make_telemetry_word(0x60CU), 7U, &t) == DSHOT_TELEMETRY_CURRENT);
    assert(t.current_a > 11.99f && t.current_a < 12.01f);

    assert(dshot_parse_telemetry(make_telemetry_word(0xC80U), 7U, &t) == DSHOT_TELEMETRY_STRESS);
    assert(t.stress_level == 0x80U);

    assert(dshot_parse_telemetry(make_telemetry_word(0xEA5U), 7U, &t) == DSHOT_TELEMETRY_STATUS);
    assert(t.status_flags == 0xA0U);
    assert(t.status_max_stress == 0x05U);

    printf("[PASS] test_parse_edt\n");
}

static void test_cmd_queue(void) {
    dshot_cmd_queue_t q;
    dshot_cmd_queue_init(&q, true);

    /* Invalid command codes. */
    assert(!dshot_cmd_request(&q, 0U));
    assert(!dshot_cmd_request(&q, 48U));
    assert(!dshot_cmd_request(NULL, DSHOT_CMD_BEEP1));

    /* Idle queue emits throttle frames with the queue's CRC polarity. */
    assert(dshot_cmd_get_frame(&q, DSHOT_3D_NEUTRAL, false) == dshot_prepare_packet(DSHOT_3D_NEUTRAL, false, true));

    assert(dshot_cmd_request(&q, DSHOT_CMD_SPIN_DIRECTION_NORMAL));
    /* A second request while one is in flight is rejected, not overwritten. */
    assert(!dshot_cmd_request(&q, DSHOT_CMD_SAVE_SETTINGS));

    uint16_t expected_cmd = dshot_prepare_packet(DSHOT_CMD_SPIN_DIRECTION_NORMAL, true, true);
    for (uint32_t i = 0; i < DSHOT_CMD_REPEAT_COUNT; i++) {
        assert(dshot_cmd_get_frame(&q, DSHOT_3D_NEUTRAL, false) == expected_cmd);
    }
    assert(q.state == DSHOT_CMD_STATE_IDLE);
    assert(dshot_cmd_get_frame(&q, DSHOT_3D_NEUTRAL, false) == dshot_prepare_packet(DSHOT_3D_NEUTRAL, false, true));

    /* Queue accepts a new command once the previous one has drained. */
    assert(dshot_cmd_request(&q, DSHOT_CMD_SAVE_SETTINGS));

    /* Unidirectional queue uses the normal CRC. */
    dshot_cmd_queue_t uni;
    dshot_cmd_queue_init(&uni, false);
    assert(dshot_cmd_request(&uni, DSHOT_CMD_BEEP1));
    assert(dshot_cmd_get_frame(&uni, 0U, false) == dshot_prepare_packet(DSHOT_CMD_BEEP1, true, false));

    printf("[PASS] test_cmd_queue\n");
}

static void test_erpm_period_to_rpm(void) {
    assert(dshot_erpm_period_to_rpm(1000U, 7U) == 8571U);
    assert(dshot_erpm_period_to_rpm(0U, 7U) == 0U);
    assert(dshot_erpm_period_to_rpm(1000U, 0U) == 0U);
    printf("[PASS] test_erpm_period_to_rpm\n");
}

int main(void) {
    printf("Running DShot Driver Unit Tests...\n");
    test_prepare_packet();
    test_decode_telemetry_frame();
    test_parse_erpm();
    test_parse_edt();
    test_cmd_queue();
    test_erpm_period_to_rpm();
    printf("All DShot Driver Tests Passed Successfully!\n");
    return 0;
}
