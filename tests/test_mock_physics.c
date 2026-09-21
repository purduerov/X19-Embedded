/**
 * @file test_mock_physics.c
 * @brief Unit tests for 6-DOF Hydrodynamic and Electrical SIL Plant Model.
 * @organization Purdue ROV
 */

#include "mock_bsp.h"
#include "mock_physics.h"
#include "mock_sensors.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* In Release builds, assert() is stripped by -DNDEBUG.
 * Variables only read inside assert() become unused.
 * Call mock_physics_get_state() inline and (void)-cast local variables
 * that are only verified via assert(). */

static void test_physics_initial_state(void) {
    mock_bsp_reset();
    mock_physics_reset();
    mock_physics_set_enabled(true);

    assert(fabsf(mock_physics_get_state()->x) < 1e-4f);
    assert(fabsf(mock_physics_get_state()->y) < 1e-4f);
    assert(fabsf(mock_physics_get_state()->z) < 1e-4f);
    assert(fabsf(mock_physics_get_state()->u) < 1e-4f);
    assert(fabsf(mock_physics_get_state()->v) < 1e-4f);
    assert(fabsf(mock_physics_get_state()->w) < 1e-4f);
    assert(fabsf(mock_physics_get_state()->q_w - 1.0f) < 1e-4f);
    assert(fabsf(mock_physics_get_state()->q_x) < 1e-4f);
    assert(fabsf(mock_physics_get_state()->q_y) < 1e-4f);
    assert(fabsf(mock_physics_get_state()->q_z) < 1e-4f);
    assert(fabsf(mock_physics_get_state()->tether_voltage_48v - 48.0f) < 1e-2f);

    for (int i = 0; i < 4; i++) {
        assert(mock_physics_get_state()->efuse_tripped[i] == false);
        assert(mock_physics_get_state()->brick_voltages_v[i] > 11.9f);
    }

    printf("[PASS] test_physics_initial_state\n");
}

static void test_buoyancy_surface_restoration(void) {
    mock_bsp_reset();
    mock_physics_reset();
    mock_physics_set_enabled(true);

    mock_physics_set_depth(1.0f);

    for (int i = 0; i < 300; i++) {
        mock_bsp_advance_time_ms(10);
    }

    assert(mock_physics_get_state()->z < 0.85f);
    assert(mock_physics_get_state()->w < -0.03f);
    assert(mock_physics_get_state()->z >= 0.0f);

    printf("[PASS] test_buoyancy_surface_restoration\n");
}

static void test_horizontal_thruster_surge(void) {
    mock_bsp_reset();
    mock_physics_reset();
    mock_physics_set_enabled(true);

    bsp_pwm_set_us(0, 1750);
    bsp_pwm_set_us(1, 1750);
    bsp_pwm_set_us(2, 1750);
    bsp_pwm_set_us(3, 1750);

    for (int i = 0; i < 200; i++) {
        mock_bsp_advance_time_ms(10);
    }

    assert(mock_physics_get_state()->u > 0.3f);
    assert(mock_physics_get_state()->x > 0.3f);

    printf("[PASS] test_horizontal_thruster_surge\n");
}

static void test_vertical_dive_and_depth_sensor_sync(void) {
    mock_bsp_reset();
    mock_physics_reset();
    mock_physics_set_enabled(true);

    bsp_pwm_set_us(4, 1750);
    bsp_pwm_set_us(5, 1750);
    bsp_pwm_set_us(6, 1750);
    bsp_pwm_set_us(7, 1750);

    for (int i = 0; i < 300; i++) {
        mock_bsp_advance_time_ms(10);
    }

    float depth_m = mock_physics_get_state()->z;
    assert(depth_m > 0.5f);

    float pressure_mbar = 0.0f;
    float temp_c = 0.0f;
    assert((int)mock_sensors_get_ms5837(&pressure_mbar, &temp_c) == 1);

    float expected_mbar = 1013.25f + (98.0665f * depth_m);
    assert(fabsf(pressure_mbar - expected_mbar) < 0.1f);

    (void)temp_c;
    (void)expected_mbar;
    (void)pressure_mbar;
    printf("[PASS] test_vertical_dive_and_depth_sensor_sync\n");
}

static void test_electrical_power_distribution(void) {
    mock_bsp_reset();
    mock_physics_reset();
    mock_physics_set_enabled(true);

    bsp_pwm_set_us(0, 1800);
    bsp_pwm_set_us(1, 1800);
    bsp_pwm_set_us(4, 1800);
    bsp_pwm_set_us(5, 1800);

    mock_bsp_advance_time_ms(20);

    assert(mock_physics_get_state()->brick_currents_a[0] > 5.0f);
    assert(mock_physics_get_state()->brick_currents_a[2] > 5.0f);
    assert(mock_physics_get_state()->brick_currents_a[1] < 1.0f);
    assert(mock_physics_get_state()->brick_currents_a[3] < 1.0f);
    assert(mock_physics_get_state()->tether_current_a > 1.5f);
    assert(mock_physics_get_state()->total_power_w > 100.0f);

    printf("[PASS] test_electrical_power_distribution\n");
}

static void test_closed_loop_depth_pid(void) {
    mock_bsp_reset();
    mock_physics_reset();
    mock_physics_set_enabled(true);

    const float target_depth_m = 1.5f;
    const float kp = 400.0f;
    const float ki = 90.0f;
    const float kd = 250.0f;
    float integral_error = 0.0f;

    for (int tick = 0; tick < 1500; tick++) {
        float current_z = mock_physics_get_state()->z;
        float current_w = mock_physics_get_state()->w;
        float error = target_depth_m - current_z;
        integral_error += error * 0.01f;
        if (integral_error > 3.0f) {
            integral_error = 3.0f;
        }
        if (integral_error < -3.0f) {
            integral_error = -3.0f;
        }

        float heave_cmd = kp * error + ki * integral_error - kd * current_w;
        float pwm_f = 1500.0f + heave_cmd;
        if (pwm_f > 1850.0f) {
            pwm_f = 1850.0f;
        }
        if (pwm_f < 1150.0f) {
            pwm_f = 1150.0f;
        }
        uint16_t pwm = (uint16_t)pwm_f;

        bsp_pwm_set_us(4, pwm);
        bsp_pwm_set_us(5, pwm);
        bsp_pwm_set_us(6, pwm);
        bsp_pwm_set_us(7, pwm);

        mock_bsp_advance_time_ms(10);
    }

    float final_z = mock_physics_get_state()->z;
    float final_w = mock_physics_get_state()->w;

    assert(fabsf(final_z - target_depth_m) < 0.05f);
    assert(fabsf(final_w) < 0.02f);

    (void)final_w;
    printf("[PASS] test_closed_loop_depth_pid (settled at %.3f m, target %.3f m)\n", (double)final_z,
           (double)target_depth_m);
}

static void test_tps25990_brick_indexing(void) {
    mock_bsp_reset();
    mock_physics_reset();
    mock_physics_set_enabled(true);

    /* Advance 1 step (10 ms) so physics updates mock sensors */
    mock_bsp_advance_time_ms(10);

    /* Brick 0 must be 5.2V logic rail (5.2V, 2.5A, 30.0C) */
    float v_in = 0.0f, v_out = 0.0f, i_out = 0.0f, temp_c = 0.0f;
    uint16_t status = 0;
    assert(mock_sensors_get_tps25990(0, &v_in, &v_out, &i_out, &temp_c, &status));
    assert(fabsf(v_out - 5.2f) < 0.01f);
    assert(fabsf(i_out - 2.5f) < 0.01f);
    assert(fabsf(temp_c - 30.0f) < 0.01f);
    assert(fabsf(v_in - 48.0f) < 0.01f);

    /* Bricks 1..4 are 12V thruster bricks */
    for (uint8_t b = 1; b <= 4; b++) {
        assert(mock_sensors_get_tps25990(b, &v_in, &v_out, &i_out, &temp_c, &status));
        assert(v_out > 11.9f);
        assert(temp_c >= 25.0f);
    }

    (void)v_in;
    (void)v_out;
    (void)i_out;
    (void)temp_c;
    (void)status;
    printf("[PASS] test_tps25990_brick_indexing\n");
}

static void test_pitched_surge_thrust_depth_coupling(void) {
    mock_bsp_reset();
    mock_physics_reset();
    mock_physics_set_enabled(true);

    /* Set vehicle depth to 1.0 m */
    mock_physics_set_depth(1.0f);

    /* Pitch vehicle nose down by 45 degrees (q_w=cos(-22.5 deg), q_y=sin(-22.5 deg)) */
    mock_physics_set_orientation(0.9238795f, 0.0f, -0.3826834f, 0.0f);

    /* Command forward surge thrust on 4 horizontal thrusters */
    bsp_pwm_set_us(0, 1750);
    bsp_pwm_set_us(1, 1750);
    bsp_pwm_set_us(2, 1750);
    bsp_pwm_set_us(3, 1750);

    /* Step simulation for 50 steps (0.5 second) */
    for (int i = 0; i < 50; i++) {
        mock_bsp_advance_time_ms(10);
    }

    /* With full 3D quaternion kinematic position rotation,
     * forward surge thrust u > 0 when pitched nose down MUST drive vehicle deeper (z > 1.0 m) */
    assert(mock_physics_get_state()->u > 0.2f);
    assert(mock_physics_get_state()->x > 0.05f);
    assert(mock_physics_get_state()->z > 1.05f);

    printf("[PASS] test_pitched_surge_thrust_depth_coupling (depth reached %.3f m from 1.0 m)\n",
           (double)mock_physics_get_state()->z);
}

static void test_added_mass_dynamics(void) {
    mock_bsp_reset();
    mock_physics_reset();
    mock_physics_set_enabled(true);

    /* Apply forward surge thrust on 4 horizontal thrusters for 1 step (10 ms) */
    bsp_pwm_set_us(0, 1750);
    bsp_pwm_set_us(1, 1750);
    bsp_pwm_set_us(2, 1750);
    bsp_pwm_set_us(3, 1750);

    mock_bsp_advance_time_ms(10);

    /* Surge apparent mass: dry mass (18.5 kg) + added mass (12.0 kg) = 30.5 kg
     * Expected u ~= 0.01268 m/s (vs 0.02090 m/s without added mass) */
    float u1 = mock_physics_get_state()->u;
    assert(u1 > 0.0120f && u1 < 0.0135f);

    /* Reset and test heave apparent mass: dry mass (18.5 kg) + added mass (24.0 kg) = 42.5 kg */
    mock_bsp_reset();
    mock_physics_reset();
    mock_physics_set_enabled(true);

    bsp_pwm_set_us(4, 1750);
    bsp_pwm_set_us(5, 1750);
    bsp_pwm_set_us(6, 1750);
    bsp_pwm_set_us(7, 1750);

    mock_bsp_advance_time_ms(10);

    /* Expected w ~= 0.01287 m/s (vs 0.02956 m/s without added mass) */
    float w1 = mock_physics_get_state()->w;
    assert(w1 > 0.0120f && w1 < 0.0135f);

    (void)u1;
    (void)w1;
    printf("[PASS] test_added_mass_dynamics (surge u=%.5f m/s, heave w=%.5f m/s)\n", (double)u1, (double)w1);
}

static void test_orientation_and_sensor_sync(void) {
    mock_bsp_reset();
    mock_physics_reset();
    mock_physics_set_enabled(true);

    /* Pitch vehicle nose down by 30 degrees: q_w = cos(-15 deg), q_y = sin(-15 deg) */
    const float q_w = 0.9659258f;
    const float q_y = -0.2588190f;
    mock_physics_set_orientation(q_w, 0.0f, q_y, 0.0f);

    /* Verify Euler angles are immediately updated without requiring a step */
    const mock_physics_state_t *st = mock_physics_get_state();
    assert(fabsf(st->pitch - (-0.5235987f)) < 1e-3f); /* -30 degrees in radians */
    assert(fabsf(st->roll) < 1e-4f);
    assert(fabsf(st->yaw) < 1e-4f);

    /* Verify IMU mock sensor is immediately synchronized */
    imu_data_t imu;
    assert(mock_sensors_get_imu(&imu));
    assert(fabsf(imu.q_w - q_w) < 1e-3f);
    assert(fabsf(imu.q_y - q_y) < 1e-3f);

    /* Verify depth sensor is immediately synchronized when setting depth */
    mock_physics_set_depth(2.5f);
    float pressure_mbar = 0.0f, temp_c = 0.0f;
    assert(mock_sensors_get_ms5837(&pressure_mbar, &temp_c));
    float expected_mbar = 1013.25f + (98.0665f * 2.5f);
    assert(fabsf(pressure_mbar - expected_mbar) < 0.1f);

    (void)temp_c;
    (void)st;
    printf("[PASS] test_orientation_and_sensor_sync\n");
}

int main(void) {
    printf("Running 6-DOF Hydrodynamic and Electrical Plant Model Tests...\n");

    test_physics_initial_state();
    test_buoyancy_surface_restoration();
    test_horizontal_thruster_surge();
    test_vertical_dive_and_depth_sensor_sync();
    test_electrical_power_distribution();
    test_closed_loop_depth_pid();
    test_tps25990_brick_indexing();
    test_pitched_surge_thrust_depth_coupling();
    test_added_mass_dynamics();
    test_orientation_and_sensor_sync();

    printf("All Hydrodynamic and Electrical Plant Model Tests Passed Successfully!\n");
    return 0;
}