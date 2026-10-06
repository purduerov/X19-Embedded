# ==============================================================================
# Purdue ROV (X19-ROV) - ST NUCLEO-F411RE / F446RE Hardware Target
# ==============================================================================

cmake_minimum_required(VERSION 3.22)
enable_language(C ASM)

add_subdirectory(cmake/stm32cubemx)

# Hardware Platform Library
add_library(f411_platform STATIC
    platform/bsp_f411.c
    platform/platform_time.c
    platform/platform_i2c.c
    platform/platform_can.c
    platform/platform_gpio.c
    platform/platform_pwm.c
    Core/Src/gpio.c
    Core/Src/can.c
    Core/Src/usart.c
)

target_include_directories(f411_platform PUBLIC
    Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../services/include
)

target_link_libraries(f411_platform PUBLIC
    stm32cubemx_f411
    rov_shared
    rov_drivers
    rov_services
    m
)

target_compile_features(f411_platform PUBLIC c_std_11)
target_compile_options(f411_platform PUBLIC -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb)

# Helper function to generate an executable combining F411 platform + node app.c
function(add_f411_node_executable TARGET_NAME)
    add_executable(${TARGET_NAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f446xx.s
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/main.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f4xx_it.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f4xx_hal_msp.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/syscalls.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sysmem.c
        ${ARGN}
    )
    target_include_directories(${TARGET_NAME} BEFORE PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    )
    target_compile_options(${TARGET_NAME} PRIVATE
        -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb
    )
    target_link_libraries(${TARGET_NAME} PRIVATE
        rov_services
        f411_platform
        STM32_Drivers_F411
        stm32cubemx_f411
        rov_shared
        rov_drivers
        m
    )
    target_link_options(${TARGET_NAME} PRIVATE
        -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb
        -T "${CMAKE_CURRENT_SOURCE_DIR}/STM32F446xx_FLASH.ld"
        -Wl,-Map=${TARGET_NAME}.map
        -Wl,--gc-sections
        -u _printf_float
    )
endfunction()

# 1. Bench Node 1 Pi Shield Target
add_f411_node_executable(f411_node1_pi_shield.elf
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node1_pi_shield/Core/Src/app.c"
)
target_include_directories(f411_node1_pi_shield.elf PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node1_pi_shield/Core/Inc"
)

# 2. Bench Node 2 Control Board Target
add_f411_node_executable(f411_node2_control_board.elf
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node2_control_board/Core/Src/app.c"
)
target_include_directories(f411_node2_control_board.elf PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node2_control_board/Core/Inc"
)

# 3. Bench Node 3 Power Slab Target
add_f411_node_executable(f411_node3_power_slab.elf
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node3_power_slab/Core/Src/app.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node3_power_slab/Core/Src/power_sequence.c"
)
target_include_directories(f411_node3_power_slab.elf PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node3_power_slab/Core/Inc"
)

# 4. R&D / Sandbox Target
if(NOT DEFINED SANDBOX_FILE OR SANDBOX_FILE STREQUAL "")
    set(SANDBOX_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../sandbox/sandbox.c")
endif()
get_filename_component(SANDBOX_FILE_ABS "${SANDBOX_FILE}" ABSOLUTE)

add_f411_node_executable(f411_sandbox.elf
    "${SANDBOX_FILE_ABS}"
)
get_filename_component(SANDBOX_DIR "${SANDBOX_FILE_ABS}" DIRECTORY)
target_include_directories(f411_sandbox.elf PRIVATE
    "${SANDBOX_DIR}"
)

# Backwards-compatible aliases
add_custom_target(testing_and_rnd.elf DEPENDS f411_sandbox.elf)
add_custom_target(bench_node1_pi_shield.elf DEPENDS f411_node1_pi_shield.elf)
add_custom_target(bench_node2_control_board.elf DEPENDS f411_node2_control_board.elf)
add_custom_target(bench_node3_power_slab.elf DEPENDS f411_node3_power_slab.elf)
