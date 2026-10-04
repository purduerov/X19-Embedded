enable_testing()
add_definitions(-DROV_UNIT_TEST -DX19_UNIT_TEST -UNDEBUG)

# Build the Software-in-the-Loop (SIL) Mock Infrastructure
# Keep the mock BSP as an object library so its timing and I2C symbols are
# available to driver-only test executables.  A static archive is not extracted
# when a driver archive introduces the first reference after the mock archive
# has already been searched.
add_library(rov_mocks OBJECT
    mocks/mock_can.c
    mocks/mock_bsp.c
    mocks/mock_bme280.c
    mocks/mock_sensors.c
    mocks/mock_physics.c
    harness/sil_clock.c
)
add_library(x19_mocks ALIAS rov_mocks)

target_include_directories(rov_mocks PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
    ${CMAKE_CURRENT_SOURCE_DIR}/../harness
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../drivers/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../services/include
)
target_link_libraries(rov_mocks PUBLIC rov_shared rov_drivers rov_services)
if(NOT WIN32 OR MINGW)
    target_link_libraries(rov_mocks PUBLIC m)
endif()

# Add test registry function that creates executables and sets CTest labels
# add_test() must come BEFORE set_tests_properties/set_property — CMake requires
# the test to be registered before its properties can be modified.
function(x19_add_node_test)
    cmake_parse_arguments(N "" "NAME;NODE;SUITE" "SOURCES;INCLUDES;LIBS" ${ARGN})
    add_executable(${N_NAME} ${N_SOURCES})
    target_include_directories(${N_NAME} PRIVATE ${N_INCLUDES})
    target_link_libraries(${N_NAME} PRIVATE ${N_LIBS})
    add_test(NAME ${N_NAME} COMMAND ${N_NAME})
    if(N_NODE)
        set_tests_properties(${N_NAME} PROPERTIES LABELS "node:${N_NODE}")
    endif()
    if(N_SUITE)
        set_property(TEST ${N_NAME} APPEND PROPERTY LABELS "suite:${N_SUITE}")
    endif()
endfunction()

# Vendored Unity test framework (ThrowTheSwitch/Unity v2.6.0, MIT)
add_library(rov_unity STATIC vendor/unity/unity.c)
target_include_directories(rov_unity PUBLIC vendor/unity)

# Unity test framework smoke check
add_executable(test_unity_smoke test_unity_smoke.c)
target_link_libraries(test_unity_smoke PRIVATE rov_unity)
add_test(NAME test_unity_smoke COMMAND test_unity_smoke)

# ROV_TEST_MAIN filter harness (ROV_TEST_MAIN / ROV_RUN_TEST macros)
add_executable(test_rov_test_main test_rov_test_main.c)
target_include_directories(test_rov_test_main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_rov_test_main PRIVATE rov_unity)
add_test(NAME test_rov_test_main COMMAND test_rov_test_main)

add_executable(test_sil_clock test_sil_clock.c)
target_include_directories(test_sil_clock PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/harness)
target_link_libraries(test_sil_clock PRIVATE rov_unity rov_mocks)
add_test(NAME test_sil_clock COMMAND test_sil_clock)

# Core Shared Library Tests
x19_add_node_test(NAME test_can_protocol SUITE shared
    SOURCES test_can_protocol.c
    LIBS rov_shared
)

x19_add_node_test(NAME test_pwm_ramp SUITE shared
    SOURCES test_pwm_ramp.c
    LIBS rov_shared
)
if(NOT WIN32 OR MINGW)
    target_link_libraries(test_pwm_ramp PRIVATE m)
endif()

x19_add_node_test(NAME test_safety SUITE safety
    SOURCES test_safety.c
    LIBS rov_shared
)

x19_add_node_test(NAME test_i2c_recovery SUITE shared
    SOURCES test_i2c_recovery.c
    LIBS rov_shared
)

x19_add_node_test(NAME test_timesync SUITE shared
    SOURCES test_timesync.c
    LIBS rov_shared rov_mocks
)

# Board Support Package (BSP) Tests
x19_add_node_test(NAME test_bsp SUITE bsp
    SOURCES test_bsp.c
    LIBS rov_mocks
)

# Peripheral Hardware Driver Unit Tests (Mocked Hardware Registers)
x19_add_node_test(NAME test_driver_bme280 SUITE driver
    SOURCES test_driver_bme280.c
    LIBS rov_mocks
)

x19_add_node_test(NAME test_driver_ms5837 SUITE driver
    SOURCES test_driver_ms5837.c
    LIBS rov_mocks
)

x19_add_node_test(NAME test_driver_ina226 SUITE driver
    SOURCES test_driver_ina226.c
    LIBS rov_mocks
)

x19_add_node_test(NAME test_driver_tcan1044 SUITE driver
    SOURCES test_driver_tcan1044.c
    LIBS rov_mocks
)

x19_add_node_test(NAME test_driver_tps25990 SUITE driver
    SOURCES test_driver_tps25990.c
    LIBS rov_mocks
)

x19_add_node_test(NAME test_driver_lsm6dsoxtr SUITE driver
    SOURCES test_driver_lsm6dsoxtr.c
    LIBS rov_mocks
)
if(NOT WIN32 OR MINGW)
    target_link_libraries(test_driver_lsm6dsoxtr PRIVATE m)
endif()

x19_add_node_test(NAME test_driver_bmi270 SUITE driver
    SOURCES test_driver_bmi270.c
    LIBS rov_mocks
)
if(NOT WIN32 OR MINGW)
    target_link_libraries(test_driver_bmi270 PRIVATE m)
endif()

x19_add_node_test(NAME test_driver_ina237 SUITE driver
    SOURCES test_driver_ina237.c
    LIBS rov_mocks
)

x19_add_node_test(NAME test_driver_tmp1075 SUITE driver
    SOURCES test_driver_tmp1075.c
    LIBS rov_mocks
)

x19_add_node_test(NAME test_driver_pmbus_brick SUITE driver
    SOURCES test_driver_pmbus_brick.c
    LIBS rov_mocks
)

x19_add_node_test(NAME test_mock_physics SUITE driver
    SOURCES test_mock_physics.c
    LIBS rov_mocks
)
if(NOT WIN32 OR MINGW)
    target_link_libraries(test_mock_physics PRIVATE m)
endif()


# Subsystem Application Node SIL Tests (Full Node Logic Execution)
x19_add_node_test(NAME test_node1_pi_shield NODE pi_shield SUITE app
    SOURCES test_node1_pi_shield.c
            ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Src/app.c
    INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Inc
            ${CMAKE_CURRENT_SOURCE_DIR}/../services/include
    LIBS rov_mocks rov_services
)

x19_add_node_test(NAME test_node2_control_board NODE control_board SUITE app
    SOURCES test_node2_control_board.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Src/app.c
    INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Inc
    LIBS rov_mocks
)
if(NOT WIN32 OR MINGW)
    target_link_libraries(test_node2_control_board PRIVATE m)
endif()

x19_add_node_test(NAME test_node3_power_slab NODE power_slab SUITE app
    SOURCES test_node3_power_slab.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Src/power_sequence.c
    INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Inc
    LIBS rov_mocks
)

# Full Multi-Node Bus Integration Simulation Test
x19_add_node_test(NAME test_multi_node_bus NODE all SUITE integration
    SOURCES test_multi_node_bus.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Src/power_sequence.c
    INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Inc
            ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Inc
            ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Inc
            ${CMAKE_CURRENT_SOURCE_DIR}/../services/include
    LIBS rov_mocks rov_services
)
if(NOT WIN32 OR MINGW)
    target_link_libraries(test_multi_node_bus PRIVATE m)
endif()

# Domain Services Test
x19_add_node_test(NAME test_services SUITE services
    SOURCES test_services.c
    INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/../services/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../shared/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../drivers/include
    LIBS rov_mocks rov_services
)

# SIL Safety, Power, Fuzz, and Burn-In Integration Tests
x19_add_node_test(NAME test_sil_safety SUITE safety
    SOURCES test_sil_safety.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Src/power_sequence.c
    INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Inc
            ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Inc
            ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Inc
    LIBS rov_mocks
)
if(NOT WIN32 OR MINGW)
    target_link_libraries(test_sil_safety PRIVATE m)
endif()

x19_add_node_test(NAME test_sil_power SUITE power
    SOURCES test_sil_power.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Src/power_sequence.c
    INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Inc
            ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Inc
            ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Inc
    LIBS rov_mocks
)
if(NOT WIN32 OR MINGW)
    target_link_libraries(test_sil_power PRIVATE m)
endif()

x19_add_node_test(NAME test_sil_fuzz SUITE fuzz
    SOURCES test_sil_fuzz.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Src/power_sequence.c
    INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Inc
            ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Inc
            ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Inc
    LIBS rov_mocks
)

x19_add_node_test(NAME test_sil_burnin SUITE burnin
    SOURCES test_sil_burnin.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Src/power_sequence.c
    INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Inc
            ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Inc
            ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Inc
    LIBS rov_mocks
)

set_tests_properties(test_sil_burnin PROPERTIES TIMEOUT 300)

x19_add_node_test(NAME test_sil_bridge_logical SUITE integration
    SOURCES test_sil_bridge_logical.c
    INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/harness
            ${CMAKE_CURRENT_SOURCE_DIR}/mocks
            ${CMAKE_CURRENT_SOURCE_DIR}/../shared/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../drivers/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../services/include
    LIBS rov_unity rov_mocks
)

# Target-readiness acceptance checks compile the real board BSPs and startup
# entrypoints against a strict fake HAL. They are expected to expose missing
# hardware integration until each target is fully implemented.
function(add_target_bsp_contract node_id node_dir test_source)
    set(bsp_source "${PROJECT_SOURCE_DIR}/nodes/${node_dir}/Core/Src/bsp.c")
    add_executable(test_target_bsp_node${node_id}
        ${test_source}
        hardware/fakes/fake_hal.c
        ${bsp_source}
    )
    target_include_directories(test_target_bsp_node${node_id} BEFORE PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/hardware/fakes
        ${PROJECT_SOURCE_DIR}/nodes/${node_dir}/Core/Inc
        ${PROJECT_SOURCE_DIR}/shared/include
        ${PROJECT_SOURCE_DIR}/drivers/include
    )
    if(MSVC)
        target_compile_options(test_target_bsp_node${node_id} PRIVATE /UROV_UNIT_TEST /UX19_UNIT_TEST)
    else()
        target_compile_options(test_target_bsp_node${node_id} PRIVATE -UROV_UNIT_TEST -UX19_UNIT_TEST)
    endif()
    add_test(NAME target_bsp_node${node_id} COMMAND test_target_bsp_node${node_id})
endfunction()

add_target_bsp_contract(1 node1_pi_shield hardware/test_target_bsp_node1.c)
add_target_bsp_contract(2 node2_control_board hardware/test_target_bsp_node2.c)
add_target_bsp_contract(3 node3_power_slab hardware/test_target_bsp_node3.c)
target_sources(test_target_bsp_node3 PRIVATE
    ${PROJECT_SOURCE_DIR}/drivers/src/ina237.c
    ${PROJECT_SOURCE_DIR}/drivers/src/tmp1075.c
)

function(add_target_startup_contract node_id node_dir)
    set(main_source "${PROJECT_SOURCE_DIR}/nodes/${node_dir}/Core/Src/main.c")
    set_source_files_properties("${main_source}" PROPERTIES
        COMPILE_DEFINITIONS "main=target_firmware_entrypoint")
    add_executable(test_target_startup_node${node_id}
        hardware/test_target_startup.c
        "${main_source}"
    )
    target_include_directories(test_target_startup_node${node_id} BEFORE PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/hardware/fakes
        ${PROJECT_SOURCE_DIR}/nodes/${node_dir}/Core/Inc
    )
    target_compile_definitions(test_target_startup_node${node_id} PRIVATE TARGET_NODE=${node_id})
    add_test(NAME target_startup_node${node_id} COMMAND test_target_startup_node${node_id})
endfunction()

add_target_startup_contract(1 node1_pi_shield)

add_test(
    NAME target_startup_node2
    COMMAND ${CMAKE_COMMAND}
        "-DMAIN_SOURCE=${PROJECT_SOURCE_DIR}/nodes/node2_control_board/Core/Src/main.c"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/hardware/check_node2_startup.cmake"
)

add_target_startup_contract(3 node3_power_slab)

# Software-in-the-Loop (SIL) Bridge Server (Host Native Only)
if(NOT CMAKE_CROSSCOMPILING)
    add_executable(sil_bridge_server
        sil_bridge_server.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node1_pi_shield/Core/Src/app.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node2_control_board/Core/Src/app.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Src/app.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../nodes/node3_power_slab/Core/Src/power_sequence.c
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

    # Host SIL Developer Sandbox / Scratchpad Target
    if(NOT DEFINED SANDBOX_FILE OR SANDBOX_FILE STREQUAL "")
        set(SANDBOX_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../sandbox/sandbox.c")
    endif()
    get_filename_component(SANDBOX_FILE_ABS "${SANDBOX_FILE}" ABSOLUTE)

    add_executable(sil_sandbox
        sil_sandbox_main.c
        ${SANDBOX_FILE_ABS}
    )
    target_link_libraries(sil_sandbox PRIVATE
        rov_mocks
        rov_shared
        rov_drivers
    )
    if(NOT WIN32 OR MINGW)
        target_link_libraries(sil_sandbox PRIVATE m)
    endif()
endif()

