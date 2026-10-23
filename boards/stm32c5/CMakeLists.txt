# ==============================================================================
# Purdue ROV (X19-ROV) - STM32C542 Vehicle PCB Hardware Target
# ==============================================================================

cmake_minimum_required(VERSION 3.22)
enable_language(C ASM)

# Hardware Platform Library for STM32C542
add_library(stm32c5_platform STATIC
    platform/bsp_stm32c5.c
    platform/platform_time.c
    platform/platform_gpio.c
    platform/platform_i2c.c
    platform/platform_can.c
    Core/Src/system_stm32c5xx.c
    Core/Src/stm32c5xx_hal_stubs.c
    Core/Src/syscalls.c
    Core/Src/sysmem.c
)

target_include_directories(stm32c5_platform PUBLIC
    Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../drivers/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../services/include
)

target_link_libraries(stm32c5_platform PUBLIC
    rov_shared
    rov_drivers
    rov_services
    m
)

target_compile_features(stm32c5_platform PUBLIC c_std_11)
target_compile_options(stm32c5_platform PUBLIC
    -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb
)

# Helper function to generate an executable combining STM32C5 platform + node app.c
function(add_stm32c5_node_executable TARGET_NAME)
    add_executable(${TARGET_NAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32c542xx.s
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/main.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/syscalls.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sysmem.c
        ${ARGN}
    )
    target_include_directories(${TARGET_NAME} BEFORE PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    )
    target_compile_options(${TARGET_NAME} PRIVATE
        -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb
    )
    target_link_libraries(${TARGET_NAME} PRIVATE
        stm32c5_platform
        rov_services
        rov_shared
        rov_drivers
        m
    )
    target_link_options(${TARGET_NAME} PRIVATE
        -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb
        -T "${CMAKE_CURRENT_SOURCE_DIR}/STM32C542xx_FLASH.ld"
        -Wl,-Map=${TARGET_NAME}.map
        -Wl,--gc-sections
        -u _printf_float
    )
endfunction()

# 1. STM32C5 Node 1 Pi Shield Target
add_stm32c5_node_executable(node1_pi_shield.elf
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node1_pi_shield/Core/Src/app.c"
)
target_include_directories(node1_pi_shield.elf PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node1_pi_shield/Core/Inc"
)

# Mirror to nodes/node1_pi_shield/ for backward compatibility with rov.toml binary_stm32c5
add_custom_command(TARGET node1_pi_shield.elf POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/nodes/node1_pi_shield"
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:node1_pi_shield.elf>" "${CMAKE_BINARY_DIR}/nodes/node1_pi_shield/node1_pi_shield.elf"
    COMMENT "Mirroring node1_pi_shield.elf into nodes/node1_pi_shield directory"
)

# Aliases
add_custom_target(stm32c5_node1_pi_shield.elf DEPENDS node1_pi_shield.elf)

# 2. STM32C5 Node 2 Control Board Target
add_stm32c5_node_executable(node2_control_board.elf
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node2_control_board/Core/Src/app.c"
)
target_include_directories(node2_control_board.elf PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node2_control_board/Core/Inc"
)

# Mirror to nodes/node2_control_board/ for backward compatibility with rov.toml binary_stm32c5
add_custom_command(TARGET node2_control_board.elf POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/nodes/node2_control_board"
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:node2_control_board.elf>" "${CMAKE_BINARY_DIR}/nodes/node2_control_board/node2_control_board.elf"
    COMMENT "Mirroring node2_control_board.elf into nodes/node2_control_board directory"
)

# Aliases
add_custom_target(stm32c5_node2_control_board.elf DEPENDS node2_control_board.elf)

# 3. STM32C5 Node 3 Power Slab Target
add_stm32c5_node_executable(node3_power_slab.elf
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node3_power_slab/Core/Src/app.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node3_power_slab/Core/Src/power_sequence.c"
)
target_include_directories(node3_power_slab.elf PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../nodes/node3_power_slab/Core/Inc"
)

# Mirror to nodes/node3_power_slab/ for backward compatibility with rov.toml binary_stm32c5
add_custom_command(TARGET node3_power_slab.elf POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/nodes/node3_power_slab"
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:node3_power_slab.elf>" "${CMAKE_BINARY_DIR}/nodes/node3_power_slab/node3_power_slab.elf"
    COMMENT "Mirroring node3_power_slab.elf into nodes/node3_power_slab directory"
)

# Aliases
add_custom_target(stm32c5_node3_power_slab.elf DEPENDS node3_power_slab.elf)

# 4. R&D / Sandbox Target
if(NOT DEFINED SANDBOX_FILE OR SANDBOX_FILE STREQUAL "" OR NOT EXISTS "${SANDBOX_FILE}")
    set(SANDBOX_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../sandbox/sandbox.c")
endif()
get_filename_component(SANDBOX_FILE_ABS "${SANDBOX_FILE}" ABSOLUTE)

add_stm32c5_node_executable(stm32c5_sandbox.elf
    "${SANDBOX_FILE_ABS}"
)
get_filename_component(SANDBOX_DIR "${SANDBOX_FILE_ABS}" DIRECTORY)
target_include_directories(stm32c5_sandbox.elf PRIVATE
    "${SANDBOX_DIR}"
)

# Aliases
add_custom_target(c5_sandbox.elf DEPENDS stm32c5_sandbox.elf)
