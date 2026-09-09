enable_testing()
add_definitions(-DROV_UNIT_TEST -DX19_UNIT_TEST)

# Build the Software-in-the-Loop (SIL) Mock Infrastructure
add_library(rov_mocks STATIC
    mocks/mock_can.c
    mocks/mock_bsp.c
    mocks/mock_sensors.c
)
add_library(x19_mocks ALIAS rov_mocks)

target_include_directories(rov_mocks PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../drivers/include
)
target_link_libraries(rov_mocks PUBLIC rov_shared rov_drivers)

# Core Shared Library Tests
add_executable(test_can_protocol test_can_protocol.c)
target_link_libraries(test_can_protocol PRIVATE rov_shared)
add_test(NAME test_can_protocol COMMAND test_can_protocol)

add_executable(test_pwm_ramp test_pwm_ramp.c)
target_link_libraries(test_pwm_ramp PRIVATE rov_shared)
if(NOT WIN32 OR MINGW)
    target_link_libraries(test_pwm_ramp PRIVATE m)
endif()
add_test(NAME test_pwm_ramp COMMAND test_pwm_ramp)

add_executable(test_safety test_safety.c)
target_link_libraries(test_safety PRIVATE rov_shared)
add_test(NAME test_safety COMMAND test_safety)

add_executable(test_i2c_recovery test_i2c_recovery.c)
target_link_libraries(test_i2c_recovery PRIVATE rov_shared)
add_test(NAME test_i2c_recovery COMMAND test_i2c_recovery)

# Peripheral Hardware Driver Unit Tests (Mocked Hardware Registers)
add_executable(test_driver_bme280 test_driver_bme280.c)
target_link_libraries(test_driver_bme280 PRIVATE rov_mocks)
add_test(NAME test_driver_bme280 COMMAND test_driver_bme280)

add_executable(test_driver_ms5837 test_driver_ms5837.c)
target_link_libraries(test_driver_ms5837 PRIVATE rov_mocks)
add_test(NAME test_driver_ms5837 COMMAND test_driver_ms5837)

add_executable(test_driver_ina226 test_driver_ina226.c)
target_link_libraries(test_driver_ina226 PRIVATE rov_mocks)
add_test(NAME test_driver_ina226 COMMAND test_driver_ina226)

add_executable(test_driver_tcan1044 test_driver_tcan1044.c)
target_link_libraries(test_driver_tcan1044 PRIVATE rov_mocks)
add_test(NAME test_driver_tcan1044 COMMAND test_driver_tcan1044)

add_executable(test_driver_tps25990 test_driver_tps25990.c)
target_link_libraries(test_driver_tps25990 PRIVATE rov_mocks)
add_test(NAME test_driver_tps25990 COMMAND test_driver_tps25990)

add_executable(test_driver_lsm6dsoxtr test_driver_lsm6dsoxtr.c)
target_link_libraries(test_driver_lsm6dsoxtr PRIVATE rov_mocks)
if(NOT WIN32 OR MINGW)
    target_link_libraries(test_driver_lsm6dsoxtr PRIVATE m)
endif()
add_test(NAME test_driver_lsm6dsoxtr COMMAND test_driver_lsm6dsoxtr)

add_executable(test_driver_bmi270 test_driver_bmi270.c)
target_link_libraries(test_driver_bmi270 PRIVATE rov_mocks)
if(NOT WIN32 OR MINGW)
    target_link_libraries(test_driver_bmi270 PRIVATE m)
endif()
add_test(NAME test_driver_bmi270 COMMAND test_driver_bmi270)

add_executable(test_driver_ina237 test_driver_ina237.c)
target_link_libraries(test_driver_ina237 PRIVATE rov_mocks)
add_test(NAME test_driver_ina237 COMMAND test_driver_ina237)

add_executable(test_driver_tmp1075 test_driver_tmp1075.c)
target_link_libraries(test_driver_tmp1075 PRIVATE rov_mocks)
add_test(NAME test_driver_tmp1075 COMMAND test_driver_tmp1075)

add_executable(test_driver_pmbus_brick test_driver_pmbus_brick.c)
target_link_libraries(test_driver_pmbus_brick PRIVATE rov_mocks)
add_test(NAME test_driver_pmbus_brick COMMAND test_driver_pmbus_brick)


# Subsystem Application Node SIL Tests (Full Node Logic Execution)
add_executable(test_node1_pi_shield
    test_node1_pi_shield.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Src/app.c
)
target_include_directories(test_node1_pi_shield PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Inc
)
target_link_libraries(test_node1_pi_shield PRIVATE rov_mocks)
add_test(NAME test_node1_pi_shield COMMAND test_node1_pi_shield)

add_executable(test_node2_control_board
    test_node2_control_board.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Src/app.c
)
target_include_directories(test_node2_control_board PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Inc
)
target_link_libraries(test_node2_control_board PRIVATE rov_mocks)
if(NOT WIN32 OR MINGW)
    target_link_libraries(test_node2_control_board PRIVATE m)
endif()
add_test(NAME test_node2_control_board COMMAND test_node2_control_board)

add_executable(test_node3_power_slab
    test_node3_power_slab.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Src/app.c
)
target_include_directories(test_node3_power_slab PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Inc
)
target_link_libraries(test_node3_power_slab PRIVATE rov_mocks)
add_test(NAME test_node3_power_slab COMMAND test_node3_power_slab)

# Full Multi-Node Bus Integration Simulation Test
add_executable(test_multi_node_bus
    test_multi_node_bus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Src/app.c
)
target_include_directories(test_multi_node_bus PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Inc
)
target_link_libraries(test_multi_node_bus PRIVATE rov_mocks)
if(NOT WIN32 OR MINGW)
    target_link_libraries(test_multi_node_bus PRIVATE m)
endif()
add_test(NAME test_multi_node_bus COMMAND test_multi_node_bus)

# Software-in-the-Loop (SIL) Bridge Server (Host Native Only)
if(NOT CMAKE_CROSSCOMPILING)
    add_executable(sil_bridge_server
        sil_bridge_server.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Src/app.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Src/app.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Src/app.c
    )
    target_include_directories(sil_bridge_server PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Inc
    )
    target_link_libraries(sil_bridge_server PRIVATE rov_mocks)
    if(WIN32)
        target_link_libraries(sil_bridge_server PRIVATE ws2_32)
    endif()
    if(NOT WIN32 OR MINGW)
        target_link_libraries(sil_bridge_server PRIVATE m)
    endif()
endif()
