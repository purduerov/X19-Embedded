/**
 * @file mock_physics.h
 * @brief 6-DOF Hydrodynamic and Electrical Plant Model for Software-in-the-Loop.
 * @organization Purdue ROV
 */

#ifndef MOCK_PHYSICS_H
#define MOCK_PHYSICS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Linear position in world frame (NED: North, East, Down in meters) */
    float x; /* Surge position (meters, forward) */
    float y; /* Sway position (meters, starboard) */
    float z; /* Heave position (depth in meters, > 0 is submerged) */

    /* Linear velocity in body frame (m/s) */
    float u; /* Surge velocity */
    float v; /* Sway velocity */
    float w; /* Heave velocity */

    /* Orientation (quaternion) */
    float q_w;
    float q_x;
    float q_y;
    float q_z;

    /* Euler angles (derived, radians) */
    float roll;
    float pitch;
    float yaw;

    /* Angular velocity in body frame (rad/s) */
    float p; /* Roll rate */
    float q; /* Pitch rate */
    float r; /* Yaw rate */

    /* Electrical state */
    float thruster_thrust_n[8];
    float thruster_currents_a[8];
    float brick_currents_a[4];
    float brick_voltages_v[4];
    float brick_temps_c[4];
    float tether_voltage_48v;
    float tether_current_a;
    float total_power_w;
    bool efuse_tripped[4];
} mock_physics_state_t;

/**
 * @brief Reset the physics and electrical model to default surface resting state.
 */
void mock_physics_reset(void);

/**
 * @brief Enable or disable physics engine simulation.
 */
void mock_physics_set_enabled(bool enabled);
bool mock_physics_is_enabled(void);

/**
 * @brief Step the physics engine by dt seconds.
 */
void mock_physics_step(float dt_s);

/**
 * @brief Query current physics state.
 */
const mock_physics_state_t *mock_physics_get_state(void);

/**
 * @brief Directly set depth (e.g. for testing depth-hold initial state).
 */
void mock_physics_set_depth(float depth_m);

/**
 * @brief Directly set vehicle orientation quaternion (e.g. for kinematic rotation verification).
 */
void mock_physics_set_orientation(float q_w, float q_x, float q_y, float q_z);

/**
 * @brief Set external water current in world frame (m/s).
 */
void mock_physics_set_water_current(float u_current, float v_current, float w_current);

/**
 * @brief Enable or disable automatic synchronization with mock_sensors.
 */
void mock_physics_set_sync_sensors(bool sync);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_PHYSICS_H */
