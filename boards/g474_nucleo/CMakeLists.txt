# ==============================================================================
# Purdue ROV (X19-ROV) - ST NUCLEO-G474RE Hardware Target
# ==============================================================================

cmake_minimum_required(VERSION 3.22)
enable_language(C ASM)

add_subdirectory(cmake/stm32cubemx)

# Hardware Platform Library
add_library(g474_platform STATIC
    platform/bsp_g474.c
    platform/platform_time.c
    platform/platform_i2c.c
    platform/platform_can.c
    platform/platform_gpio.c
    platform/platform_pwm.c
    Core/Src/stm32g4xx_it.c
    Core/Src/stm32g4xx_hal_msp.c
    Core/Src/sysmem.c
    Core/Src/syscalls.c
    startup_stm32g474xx.s
)

target_include_directories(g474_platform PUBLIC
    Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../services/include
)

target_link_libraries(g474_platform PUBLIC
    stm32cubemx_g474
    rov_shared
    rov_drivers
    rov_services
    m
)

target_compile_features(g474_platform PUBLIC c_std_11)
target_compile_options(g474_platform PUBLIC -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb)

# Helper function to generate an executable combining G474 platform + node app sources
function(add_g474_node_executable TARGET_NAME)
    add_executable(${TARGET_NAME}
        Core/Src/main.c
        ${ARGN}
    )
    target_include_directories(${TARGET_NAME} BEFORE PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    )
    target_compile_options(${TARGET_NAME} PRIVATE
        -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb
    )
    target_link_libraries(${TARGET_NAME} PRIVATE
        g474_platform
        STM32_Drivers_G474
        stm32cubemx_g474
        rov_shared
        rov_drivers
        rov_services
        m
    )
    target_link_options(${TARGET_NAME} PRIVATE
        -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb
        -T "${CMAKE_CURRENT_SOURCE_DIR}/STM32G474xx_FLASH.ld"
        -Wl,-Map=${TARGET_NAME}.map
        -Wl,--gc-sections
        -u _printf_float
    )
endfunction()

# 1. Bench Node 1 Pi Shield Target
add_g474_node_executable(g474_node1_pi_shield.elf
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node1_pi_shield/Core/Src/app.c"
)
target_include_directories(g474_node1_pi_shield.elf PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node1_pi_shield/Core/Inc"
)

# 2. Bench Node 2 Control Board Target
add_g474_node_executable(g474_node2_control_board.elf
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node2_control_board/Core/Src/app.c"
)
target_include_directories(g474_node2_control_board.elf PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node2_control_board/Core/Inc"
)

# Backwards compatible alias: node2_control_board.elf -> g474_node2_control_board.elf
add_custom_target(node2_control_board.elf DEPENDS g474_node2_control_board.elf)

# 3. Bench Node 3 Power Slab Target
add_g474_node_executable(g474_node3_power_slab.elf
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node3_power_slab/Core/Src/app.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node3_power_slab/Core/Src/power_sequence.c"
)
target_include_directories(g474_node3_power_slab.elf PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node3_power_slab/Core/Inc"
)

# 4. R&D / Sandbox Target
add_g474_node_executable(g474_sandbox.elf
    "${CMAKE_CURRENT_SOURCE_DIR}/../../sandbox/sandbox.c"
)
target_include_directories(g474_sandbox.elf PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../sandbox"
)

# 5. Diagnostic I2C Scanner
add_g474_node_executable(g474_i2c_scanner.elf
    "Core/Src/diagnostic_i2c_scan.c"
)
