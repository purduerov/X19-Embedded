/**
 * @file dshot.h
 * @brief Digital DShot & Bidirectional Telemetry (BDShot/EDT) Driver.
 * @organization Purdue ROV
 */

#ifndef DSHOT_H
#define DSHOT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DSHOT_MIN_THROTTLE 48U
#define DSHOT_MAX_THROTTLE 2047U
#define DSHOT_VALUE_MASK   0x07FFU /* 11-bit throttle/command field */
#define DSHOT_3D_NEUTRAL   1048U   /* 3D mode: armed, motor stopped */

/* eRPM period reported by the ESC when the motor is stopped (mantissa 0x1FF, exponent 7). */
#define DSHOT_ERPM_PERIOD_STOPPED 65408U

/* DShot Special Commands */
#define DSHOT_CMD_MOTOR_STOP                 0U
#define DSHOT_CMD_BEEP1                      1U
#define DSHOT_CMD_BEEP2                      2U
#define DSHOT_CMD_BEEP3                      3U
#define DSHOT_CMD_BEEP4                      4U
#define DSHOT_CMD_BEEP5                      5U
#define DSHOT_CMD_ESC_INFO                   6U
#define DSHOT_CMD_SPIN_DIRECTION_1           7U
#define DSHOT_CMD_SPIN_DIRECTION_2           8U
#define DSHOT_CMD_3D_MODE_OFF                9U
#define DSHOT_CMD_3D_MODE_ON                 10U
#define DSHOT_CMD_SAVE_SETTINGS              12U
#define DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE  13U
#define DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE 14U
#define DSHOT_CMD_SPIN_DIRECTION_NORMAL      20U
#define DSHOT_CMD_SPIN_DIRECTION_REVERSED    21U

/* Telemetry Types */
typedef enum {
    DSHOT_TELEMETRY_NONE = 0,
    DSHOT_TELEMETRY_ERPM,
    DSHOT_TELEMETRY_TEMPERATURE,
    DSHOT_TELEMETRY_VOLTAGE,
    DSHOT_TELEMETRY_CURRENT,
    DSHOT_TELEMETRY_STRESS,
    DSHOT_TELEMETRY_STATUS,
    DSHOT_TELEMETRY_DEBUG
} dshot_telemetry_type_t;

typedef struct {
    uint32_t erpm;
    uint32_t rpm; /* Mechanical RPM for specified pole pairs */
    float voltage_v;
    float current_a;
    int16_t temperature_c;
    uint8_t stress_level;      /* Last EDT stress metric [0-255] */
    uint8_t status_flags;      /* Last EDT status flags: 0x80 alert, 0x40 warning, 0x20 error */
    uint8_t status_max_stress; /* Last EDT status max-stress nibble [0-15] */
    dshot_telemetry_type_t last_type;
} dshot_telemetry_t;

/**
 * @brief Prepare 16-bit DShot packet from 11-bit throttle value, telemetry request, and CRC.
 * @param value Throttle (48..2047) or special command (1..47); masked to 11 bits.
 * @param telemetry Telemetry request bit.
 * @param invert_crc true for bidirectional DShot (BDShot), false for unidirectional.
 */
uint16_t dshot_prepare_packet(uint16_t value, bool telemetry, bool invert_crc);

/**
 * @brief Decode 21-bit GCR telemetry into 16-bit telemetry frame.
 * @param raw_21_bits 21-bit sampled GCR stream.
 * @param out_telemetry_16 Pointer to destination 16-bit word.
 * @return true if GCR and CRC are valid, false otherwise.
 */
bool dshot_decode_telemetry_frame(uint32_t raw_21_bits, uint16_t *out_telemetry_16);

/**
 * @brief Parse a valid 16-bit telemetry frame into telemetry struct.
 * @param frame_16 Decoded 16-bit telemetry word.
 * @param motor_pole_pairs Number of motor pole pairs (7 for T200); 0 leaves rpm at 0.
 * @param telem Pointer to telemetry state struct to update.
 * @return Parsed telemetry type.
 */
dshot_telemetry_type_t dshot_parse_telemetry(uint16_t frame_16, uint8_t motor_pole_pairs, dshot_telemetry_t *telem);

#define DSHOT_CMD_REPEAT_COUNT 10U

typedef enum { DSHOT_CMD_STATE_IDLE = 0, DSHOT_CMD_STATE_SENDING } dshot_cmd_state_t;

typedef struct {
    dshot_cmd_state_t state;
    uint16_t command_frame;
    uint8_t repeat_count;
    bool invert_crc; /* CRC polarity applied to every frame this queue emits */
} dshot_cmd_queue_t;

/**
 * @brief Initialize a non-blocking DShot command queue.
 * @param invert_crc true when the ESC runs bidirectional DShot (inverted CRC).
 */
void dshot_cmd_queue_init(dshot_cmd_queue_t *queue, bool invert_crc);

/**
 * @brief Request transmission of a DShot special command (1..47).
 * Prepares the frame with Telemetry bit = 1 and sets repeat counter to DSHOT_CMD_REPEAT_COUNT.
 * @return true if queued; false if the code is invalid or another command is still in flight.
 */
bool dshot_cmd_request(dshot_cmd_queue_t *queue, uint8_t cmd_code);

/**
 * @brief Get the next 16-bit frame to transmit for this motor cycle.
 * If a command is active, returns the command frame and decrements repeat counter.
 * Otherwise returns the normal packed throttle frame.
 */
uint16_t dshot_cmd_get_frame(dshot_cmd_queue_t *queue, uint16_t throttle, bool telemetry);

/**
 * @brief Convert raw eRPM period in microseconds to mechanical motor RPM (7 pole pairs for T200).
 * @return 0 if period_us or pole_pairs is 0.
 */
uint32_t dshot_erpm_period_to_rpm(uint32_t period_us, uint8_t pole_pairs);

#ifdef __cplusplus
}
#endif

#endif /* DSHOT_H */
