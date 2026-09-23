/**
 * @file mock_physics.c
 * @brief 6-DOF Hydrodynamic and Electrical Plant Model Implementation.
 * @organization Purdue ROV
 */

#include "mock_physics.h"
#include "mock_bsp.h"
#include "mock_sensors.h"
#include <math.h>
#include <string.h>

#define PHYS_PI 3.14159265358979323846f

/* ROV Physical Characteristics */
#define ROV_MASS_KG         18.5f
#define ROV_BUOYANCY_N      184.0f /* Net positive buoyancy = ~2.58 N (Mass * g = 181.42 N) */
#define ROV_GRAVITY_MPS2    9.80665f
#define ROV_WEIGHT_N        (ROV_MASS_KG * ROV_GRAVITY_MPS2)
#define ROV_METACENTRIC_H_M 0.035f /* Distance between CB and CG for roll/pitch stability */

#define ROV_INERTIA_IX_KGM2 0.25f
#define ROV_INERTIA_IY_KGM2 0.35f
#define ROV_INERTIA_IZ_KGM2 0.40f

/* Hydrodynamic Added Mass (M_A) and Added Inertia */
#define ROV_ADDED_MASS_X_KG 12.0f /* Surge added mass (kg) */
#define ROV_ADDED_MASS_Y_KG 15.0f /* Sway added mass (kg) */
#define ROV_ADDED_MASS_Z_KG 24.0f /* Heave added mass (kg) */

#define ROV_ADDED_INERTIA_IX_KGM2 0.15f /* Roll added inertia (kg*m^2) */
#define ROV_ADDED_INERTIA_IY_KGM2 0.20f /* Pitch added inertia (kg*m^2) */
#define ROV_ADDED_INERTIA_IZ_KGM2 0.25f /* Yaw added inertia (kg*m^2) */

/* Total Apparent Mass and Inertia (Rigid Body + Added Mass) */
#define ROV_TOTAL_MASS_X_KG (ROV_MASS_KG + ROV_ADDED_MASS_X_KG)
#define ROV_TOTAL_MASS_Y_KG (ROV_MASS_KG + ROV_ADDED_MASS_Y_KG)
#define ROV_TOTAL_MASS_Z_KG (ROV_MASS_KG + ROV_ADDED_MASS_Z_KG)

#define ROV_TOTAL_INERTIA_IX_KGM2 (ROV_INERTIA_IX_KGM2 + ROV_ADDED_INERTIA_IX_KGM2)
#define ROV_TOTAL_INERTIA_IY_KGM2 (ROV_INERTIA_IY_KGM2 + ROV_ADDED_INERTIA_IY_KGM2)
#define ROV_TOTAL_INERTIA_IZ_KGM2 (ROV_INERTIA_IZ_KGM2 + ROV_ADDED_INERTIA_IZ_KGM2)

/* Geometry lever arms */
#define ROV_ARM_HORIZ_X_M 0.18f
#define ROV_ARM_HORIZ_Y_M 0.15f
#define ROV_ARM_VERT_X_M  0.16f
#define ROV_ARM_VERT_Y_M  0.13f

static mock_physics_state_t g_phys;
static bool g_physics_enabled = false;
static bool g_sync_sensors = true;
static float g_water_current[3] = {0.0f, 0.0f, 0.0f};

static void compute_thruster(uint16_t pwm_us, float *thrust_n, float *current_a) {
    if (pwm_us < 1100) {
        pwm_us = 1100;
    }
    if (pwm_us > 1900) {
        pwm_us = 1900;
    }

    if (pwm_us > 1525) {
        float norm = (float)(pwm_us - 1500) / 400.0f;
        *thrust_n = 35.0f * norm * norm;
        *current_a = 0.2f + 14.8f * powf(norm, 1.8f);
    } else if (pwm_us < 1475) {
        float norm = (float)(1500 - pwm_us) / 400.0f;
        *thrust_n = -28.0f * norm * norm;
        *current_a = 0.2f + 11.8f * powf(norm, 1.8f);
    } else {
        *thrust_n = 0.0f;
        *current_a = 0.2f;
    }
}

void mock_physics_reset(void) {
    memset(&g_phys, 0, sizeof(g_phys));
    g_phys.q_w = 1.0f;
    g_phys.tether_voltage_48v = 48.0f;
    for (int i = 0; i < 4; i++) {
        g_phys.brick_voltages_v[i] = 12.0f;
        g_phys.brick_temps_c[i] = 25.0f;
        g_phys.efuse_tripped[i] = false;
    }
    g_water_current[0] = 0.0f;
    g_water_current[1] = 0.0f;
    g_water_current[2] = 0.0f;
}

void mock_physics_set_enabled(bool enabled) {
    g_physics_enabled = enabled;
}

bool mock_physics_is_enabled(void) {
    return g_physics_enabled;
}

void mock_physics_set_sync_sensors(bool sync) {
    g_sync_sensors = sync;
}

const mock_physics_state_t *mock_physics_get_state(void) {
    return &g_phys;
}

static void compute_euler_angles(void) {
    g_phys.roll = atan2f(2.0f * (g_phys.q_w * g_phys.q_x + g_phys.q_y * g_phys.q_z),
                         1.0f - 2.0f * (g_phys.q_x * g_phys.q_x + g_phys.q_y * g_phys.q_y));
    float sin_pitch = 2.0f * (g_phys.q_w * g_phys.q_y - g_phys.q_z * g_phys.q_x);
    if (fabsf(sin_pitch) >= 1.0f) {
        g_phys.pitch = copysignf(PHYS_PI / 2.0f, sin_pitch);
    } else {
        g_phys.pitch = asinf(sin_pitch);
    }
    g_phys.yaw = atan2f(2.0f * (g_phys.q_w * g_phys.q_z + g_phys.q_x * g_phys.q_y),
                        1.0f - 2.0f * (g_phys.q_y * g_phys.q_y + g_phys.q_z * g_phys.q_z));
}

void mock_physics_set_depth(float depth_m) {
    if (depth_m < 0.0f) {
        depth_m = 0.0f;
    }
    g_phys.z = depth_m;
    if (g_sync_sensors) {
        float pressure_mbar = 1013.25f + (98.0665f * g_phys.z);
        mock_sensors_set_ms5837(pressure_mbar, 18.0f);
    }
}

void mock_physics_set_orientation(float q_w, float q_x, float q_y, float q_z) {
    float norm = sqrtf(q_w * q_w + q_x * q_x + q_y * q_y + q_z * q_z);
    if (norm > 1e-6f) {
        g_phys.q_w = q_w / norm;
        g_phys.q_x = q_x / norm;
        g_phys.q_y = q_y / norm;
        g_phys.q_z = q_z / norm;
        compute_euler_angles();
        if (g_sync_sensors) {
            imu_data_t imu;
            imu.q_w = g_phys.q_w;
            imu.q_x = g_phys.q_x;
            imu.q_y = g_phys.q_y;
            imu.q_z = g_phys.q_z;
            imu.gyro_x_dps = g_phys.p * (180.0f / PHYS_PI);
            imu.gyro_y_dps = g_phys.q * (180.0f / PHYS_PI);
            imu.gyro_z_dps = g_phys.r * (180.0f / PHYS_PI);
            imu.status = 3;
            mock_sensors_set_imu(&imu);
        }
    }
}

void mock_physics_set_water_current(float u_current, float v_current, float w_current) {
    g_water_current[0] = u_current;
    g_water_current[1] = v_current;
    g_water_current[2] = w_current;
}

void mock_physics_step(float dt_s) {
    if (!g_physics_enabled || dt_s <= 0.0f) {
        return;
    }

    /* 1. Ingest commanded thruster PWMs and compute thrust/current */
    for (int i = 0; i < 8; i++) {
        uint16_t pwm = bsp_pwm_get_us((uint8_t)i);
        compute_thruster(pwm, &g_phys.thruster_thrust_n[i], &g_phys.thruster_currents_a[i]);
    }

    /* 2. Map thrusters to 4x 12V 300W DC-DC converter bricks */
    float total_12v_power = 0.0f;
    for (int b = 0; b < 4; b++) {
        int t1 = b * 2;
        int t2 = b * 2 + 1;
        g_phys.brick_currents_a[b] = g_phys.thruster_currents_a[t1] + g_phys.thruster_currents_a[t2];

        /* Overcurrent trip threshold: 25.0 A */
        if (g_phys.brick_currents_a[b] > 25.0f) {
            g_phys.efuse_tripped[b] = true;
        }

        /* Load regulation voltage droop */
        g_phys.brick_voltages_v[b] = 12.0f - 0.02f * g_phys.brick_currents_a[b];
        total_12v_power += g_phys.brick_voltages_v[b] * g_phys.brick_currents_a[b];

        /* Thermal model */
        float p_loss = g_phys.brick_currents_a[b] * g_phys.brick_currents_a[b] * 0.008f;
        float d_temp = (p_loss - (g_phys.brick_temps_c[b] - 25.0f) / 15.0f) / 50.0f;
        g_phys.brick_temps_c[b] += d_temp * dt_s;
    }

    /* Tether 48V input current (92% efficiency + 20W logic) */
    g_phys.total_power_w = (total_12v_power / 0.92f) + 20.0f;
    g_phys.tether_current_a = g_phys.total_power_w / g_phys.tether_voltage_48v;

    /* 3. 8-Thruster Kinematic Force Allocation (Heavy Configuration) */
    const float k_diag = 0.70710678f; /* cos(45 deg) */

    /* Horizontal Thrusters: T0 (FR), T1 (FL), T2 (RR), T3 (RL) */
    float t0 = g_phys.thruster_thrust_n[0];
    float t1 = g_phys.thruster_thrust_n[1];
    float t2 = g_phys.thruster_thrust_n[2];
    float t3 = g_phys.thruster_thrust_n[3];

    float f_surge_thrusters = k_diag * (t0 + t1 + t2 + t3);
    float f_sway_thrusters = k_diag * (-t0 + t1 + t2 - t3);
    float m_yaw_thrusters = ROV_ARM_HORIZ_X_M * (-t0 + t1 - t2 + t3);

    /* Vertical Thrusters: T4 (FR), T5 (FL), T6 (RR), T7 (RL) */
    float t4 = g_phys.thruster_thrust_n[4];
    float t5 = g_phys.thruster_thrust_n[5];
    float t6 = g_phys.thruster_thrust_n[6];
    float t7 = g_phys.thruster_thrust_n[7];

    /* Heave force: Positive thrust drives vehicle DOWN into water */
    float f_heave_thrusters = (t4 + t5 + t6 + t7);
    float m_roll_thrusters = ROV_ARM_VERT_Y_M * (-t4 + t5 - t6 + t7);
    float m_pitch_thrusters = ROV_ARM_VERT_X_M * (-t4 - t5 + t6 + t7);

    /* 4. Buoyancy & Restoring Forces (NED coordinates, z > 0 is submerged) */
    float f_net_heave = f_heave_thrusters;
    float m_restoring_roll = 0.0f;
    float m_restoring_pitch = 0.0f;

    if (g_phys.z > 0.001f) {
        /* Submerged: Net positive buoyancy restores upward (-z direction) */
        float net_buoyancy = ROV_BUOYANCY_N - ROV_WEIGHT_N; /* ~ +2.0 N upward */
        f_net_heave -= net_buoyancy;

        /* Metacentric righting moment */
        m_restoring_roll = -ROV_BUOYANCY_N * ROV_METACENTRIC_H_M * sinf(g_phys.roll);
        m_restoring_pitch = -ROV_BUOYANCY_N * ROV_METACENTRIC_H_M * sinf(g_phys.pitch);
    }

    /* 5. Hydrodynamic Quadratic + Linear Drag (relative to water current) */
    float rel_u = g_phys.u - g_water_current[0];
    float rel_v = g_phys.v - g_water_current[1];
    float rel_w = g_phys.w - g_water_current[2];

    float f_drag_u = -(12.0f * rel_u + 35.0f * rel_u * fabsf(rel_u));
    float f_drag_v = -(15.0f * rel_v + 45.0f * rel_v * fabsf(rel_v));
    float f_drag_w = -(20.0f * rel_w + 60.0f * rel_w * fabsf(rel_w));

    float m_drag_p = -(1.5f * g_phys.p + 3.0f * g_phys.p * fabsf(g_phys.p));
    float m_drag_q = -(2.0f * g_phys.q + 4.0f * g_phys.q * fabsf(g_phys.q));
    float m_drag_r = -(2.5f * g_phys.r + 5.0f * g_phys.r * fabsf(g_phys.r));

    /* 6. Linear Equations of Motion (F = M_total * a) */
    float du = (f_surge_thrusters + f_drag_u) / ROV_TOTAL_MASS_X_KG;
    float dv = (f_sway_thrusters + f_drag_v) / ROV_TOTAL_MASS_Y_KG;
    float dw = (f_net_heave + f_drag_w) / ROV_TOTAL_MASS_Z_KG;

    g_phys.u += du * dt_s;
    g_phys.v += dv * dt_s;
    g_phys.w += dw * dt_s;

    /* Surface constraint: vehicle cannot float above water line */
    if (g_phys.z <= 0.0f && g_phys.w < 0.0f) {
        g_phys.w = 0.0f;
    }

    /* 7. Rotational Equations of Motion (T = I_total * alpha) */
    float dp = (m_roll_thrusters + m_restoring_roll + m_drag_p) / ROV_TOTAL_INERTIA_IX_KGM2;
    float dq = (m_pitch_thrusters + m_restoring_pitch + m_drag_q) / ROV_TOTAL_INERTIA_IY_KGM2;
    float dr = (m_yaw_thrusters + m_drag_r) / ROV_TOTAL_INERTIA_IZ_KGM2;

    g_phys.p += dp * dt_s;
    g_phys.q += dq * dt_s;
    g_phys.r += dr * dt_s;

    /* 8. Position Integration (Full 3D quaternion kinematic rotation: v^n = q (x) v^b (x) q*) */
    float qw = g_phys.q_w;
    float qx = g_phys.q_x;
    float qy = g_phys.q_y;
    float qz = g_phys.q_z;

    /* Direction cosine matrix mapping body frame velocities to navigation/world frame (NED) */
    float r11 = 1.0f - 2.0f * (qy * qy + qz * qz);
    float r12 = 2.0f * (qx * qy - qw * qz);
    float r13 = 2.0f * (qx * qz + qw * qy);

    float r21 = 2.0f * (qx * qy + qw * qz);
    float r22 = 1.0f - 2.0f * (qx * qx + qz * qz);
    float r23 = 2.0f * (qy * qz - qw * qx);

    float r31 = 2.0f * (qx * qz - qw * qy);
    float r32 = 2.0f * (qy * qz + qw * qx);
    float r33 = 1.0f - 2.0f * (qx * qx + qy * qy);

    float dx = r11 * g_phys.u + r12 * g_phys.v + r13 * g_phys.w;
    float dy = r21 * g_phys.u + r22 * g_phys.v + r23 * g_phys.w;
    float dz = r31 * g_phys.u + r32 * g_phys.v + r33 * g_phys.w;

    g_phys.x += dx * dt_s;
    g_phys.y += dy * dt_s;
    g_phys.z += dz * dt_s;

    if (g_phys.z < 0.0f) {
        g_phys.z = 0.0f;
    }

    /* 9. Quaternion Integration: q_dot = 0.5 * q * [0, p, q, r] */
    float q_w = g_phys.q_w;
    float q_x = g_phys.q_x;
    float q_y = g_phys.q_y;
    float q_z = g_phys.q_z;

    float dq_w = 0.5f * (-q_x * g_phys.p - q_y * g_phys.q - q_z * g_phys.r);
    float dq_x = 0.5f * (q_w * g_phys.p + q_y * g_phys.r - q_z * g_phys.q);
    float dq_y = 0.5f * (q_w * g_phys.q - q_x * g_phys.r + q_z * g_phys.p);
    float dq_z = 0.5f * (q_w * g_phys.r + q_x * g_phys.q - q_y * g_phys.p);

    q_w += dq_w * dt_s;
    q_x += dq_x * dt_s;
    q_y += dq_y * dt_s;
    q_z += dq_z * dt_s;

    /* Normalize quaternion */
    float norm = sqrtf(q_w * q_w + q_x * q_x + q_y * q_y + q_z * q_z);
    if (norm > 1e-6f) {
        g_phys.q_w = q_w / norm;
        g_phys.q_x = q_x / norm;
        g_phys.q_y = q_y / norm;
        g_phys.q_z = q_z / norm;
    }

    /* Extract Euler angles from quaternion */
    compute_euler_angles();

    /* 10. Automatically synchronize with synthetic sensor layer */
    if (g_sync_sensors) {
        /* MS5837 Hydrostatic Depth: 1013.25 mbar + 98.0665 mbar per meter */
        float pressure_mbar = 1013.25f + (98.0665f * g_phys.z);
        mock_sensors_set_ms5837(pressure_mbar, 18.0f);

        /* BMI270 / LSM6DSOXTR IMU Data */
        imu_data_t imu;
        imu.q_w = g_phys.q_w;
        imu.q_x = g_phys.q_x;
        imu.q_y = g_phys.q_y;
        imu.q_z = g_phys.q_z;
        imu.gyro_x_dps = g_phys.p * (180.0f / PHYS_PI);
        imu.gyro_y_dps = g_phys.q * (180.0f / PHYS_PI);
        imu.gyro_z_dps = g_phys.r * (180.0f / PHYS_PI);
        imu.status = 3;
        mock_sensors_set_imu(&imu);

        /* TPS25990 Bricks: brick 0 is 5.2V logic rail; bricks 1..4 are 12V thruster bricks */
        mock_sensors_set_tps25990(0, g_phys.tether_voltage_48v, 5.2f, 2.5f, 30.0f, 0x0000);
        for (uint8_t i = 0; i < 4; i++) {
            uint16_t status = g_phys.efuse_tripped[i] ? 0x0800 : 0x0000;
            mock_sensors_set_tps25990((uint8_t)(i + 1), g_phys.tether_voltage_48v, g_phys.brick_voltages_v[i],
                                      g_phys.brick_currents_a[i], g_phys.brick_temps_c[i], status);
        }

        /* INA226 12V bus monitor & INA237 48V tether monitor */
        float total_12v_cur = g_phys.brick_currents_a[0] + g_phys.brick_currents_a[1] + g_phys.brick_currents_a[2] +
                              g_phys.brick_currents_a[3];
        mock_sensors_set_ina226(g_phys.brick_voltages_v[0], total_12v_cur);
    }
}
