/**
 * @file mock_sensors.c
 * @brief Synthetic Sensor Injection Implementation.
 * @organization Purdue ROV
 */

#include "mock_sensors.h"
#include <string.h>

static struct {
    float pressure_hpa;
    float humidity_pct;
    float temp_c;
    bool valid;
} g_mock_bme280;

static struct {
    float pressure_mbar;
    float temp_c;
    bool valid;
} g_mock_ms5837;

static struct {
    imu_data_t data;
    bool valid;
} g_mock_imu;

static struct {
    float voltage_v;
    float current_a;
    bool valid;
} g_mock_ina226;

static struct {
    float v_in;
    float v_out;
    float i_out;
    float temp_c;
    uint16_t status;
    bool valid;
} g_mock_tps[5];

static struct {
    float temp_c;
    bool valid;
} g_mock_tmp1075;

void mock_sensors_reset(void) {
    /* Nominal atmospheric defaults */
    g_mock_bme280.pressure_hpa = 1013.25f;
    g_mock_bme280.humidity_pct = 35.0f;
    g_mock_bme280.temp_c = 24.0f;
    g_mock_bme280.valid = true;

    g_mock_ms5837.pressure_mbar = 1013.25f;
    g_mock_ms5837.temp_c = 18.0f;
    g_mock_ms5837.valid = true;

    g_mock_imu.data.q_w = 1.0f;
    g_mock_imu.data.q_x = 0.0f;
    g_mock_imu.data.q_y = 0.0f;
    g_mock_imu.data.q_z = 0.0f;
    g_mock_imu.data.gyro_x_dps = 0.0f;
    g_mock_imu.data.gyro_y_dps = 0.0f;
    g_mock_imu.data.gyro_z_dps = 0.0f;
    g_mock_imu.data.status = 3;
    g_mock_imu.valid = true;

    g_mock_ina226.voltage_v = 5.2f;
    g_mock_ina226.current_a = 1.5f;
    g_mock_ina226.valid = true;

    for (int i = 0; i < 5; i++) {
        g_mock_tps[i].v_in = 48.0f;
        g_mock_tps[i].v_out = (i == 0) ? 5.2f : 12.0f;
        g_mock_tps[i].i_out = 2.0f;
        g_mock_tps[i].temp_c = 35.0f;
        g_mock_tps[i].status = 0;
        g_mock_tps[i].valid = true;
    }

    g_mock_tmp1075.temp_c = 25.0f;
    g_mock_tmp1075.valid = true;
}

void mock_sensors_set_bme280(float pressure_hpa, float humidity_pct, float temp_c) {
    g_mock_bme280.pressure_hpa = pressure_hpa;
    g_mock_bme280.humidity_pct = humidity_pct;
    g_mock_bme280.temp_c = temp_c;
    g_mock_bme280.valid = true;
}

bool mock_sensors_get_bme280(float *pressure_hpa, float *humidity_pct, float *temp_c) {
    if (!g_mock_bme280.valid)
        return false;
    if (pressure_hpa)
        *pressure_hpa = g_mock_bme280.pressure_hpa;
    if (humidity_pct)
        *humidity_pct = g_mock_bme280.humidity_pct;
    if (temp_c)
        *temp_c = g_mock_bme280.temp_c;
    return true;
}

void mock_sensors_set_ms5837(float pressure_mbar, float temp_c) {
    g_mock_ms5837.pressure_mbar = pressure_mbar;
    g_mock_ms5837.temp_c = temp_c;
    g_mock_ms5837.valid = true;
}

bool mock_sensors_get_ms5837(float *pressure_mbar, float *temp_c) {
    if (!g_mock_ms5837.valid)
        return false;
    if (pressure_mbar)
        *pressure_mbar = g_mock_ms5837.pressure_mbar;
    if (temp_c)
        *temp_c = g_mock_ms5837.temp_c;
    return true;
}

void mock_sensors_set_imu(const imu_data_t *data) {
    if (data) {
        g_mock_imu.data = *data;
        g_mock_imu.valid = true;
    }
}

bool mock_sensors_get_imu(imu_data_t *data) {
    if (!g_mock_imu.valid || !data)
        return false;
    *data = g_mock_imu.data;
    return true;
}

void mock_sensors_set_ina226(float voltage_v, float current_a) {
    g_mock_ina226.voltage_v = voltage_v;
    g_mock_ina226.current_a = current_a;
    g_mock_ina226.valid = true;
}

bool mock_sensors_get_ina226(float *voltage_v, float *current_a) {
    if (!g_mock_ina226.valid)
        return false;
    if (voltage_v)
        *voltage_v = g_mock_ina226.voltage_v;
    if (current_a)
        *current_a = g_mock_ina226.current_a;
    return true;
}

void mock_sensors_set_tps25990(uint8_t brick_idx, float v_in, float v_out, float i_out, float temp_c, uint16_t status) {
    if (brick_idx < 5) {
        g_mock_tps[brick_idx].v_in = v_in;
        g_mock_tps[brick_idx].v_out = v_out;
        g_mock_tps[brick_idx].i_out = i_out;
        g_mock_tps[brick_idx].temp_c = temp_c;
        g_mock_tps[brick_idx].status = status;
        g_mock_tps[brick_idx].valid = true;
    }
}

bool mock_sensors_get_tps25990(uint8_t brick_idx, float *v_in, float *v_out, float *i_out, float *temp_c,
                               uint16_t *status) {
    if (brick_idx >= 5 || !g_mock_tps[brick_idx].valid)
        return false;
    if (v_in)
        *v_in = g_mock_tps[brick_idx].v_in;
    if (v_out)
        *v_out = g_mock_tps[brick_idx].v_out;
    if (i_out)
        *i_out = g_mock_tps[brick_idx].i_out;
    if (temp_c)
        *temp_c = g_mock_tps[brick_idx].temp_c;
    if (status)
        *status = g_mock_tps[brick_idx].status;
    return true;
}

void mock_sensors_set_tmp1075(float temp_c) {
    g_mock_tmp1075.temp_c = temp_c;
    g_mock_tmp1075.valid = true;
}

bool mock_sensors_get_tmp1075(float *temp_c) {
    if (!g_mock_tmp1075.valid)
        return false;
    if (temp_c)
        *temp_c = g_mock_tmp1075.temp_c;
    return true;
}

