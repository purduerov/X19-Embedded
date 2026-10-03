cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME testing_and_rnd)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES SUFFIX ".elf")

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Src/bsp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Src/can_f4.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../node1_pi_shield/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../node2_control_board/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../node3_power_slab/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../node3_power_slab/Core/Src/power_sequence.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../drivers/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../node1_pi_shield/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../node2_control_board/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../node3_power_slab/Core/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T "${CMAKE_CURRENT_SOURCE_DIR}/STM32F446xx_FLASH.ld"
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -Wl,--gc-sections
    -u _printf_float
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    rov_shared
    rov_drivers
    m
)

# ==============================================================================
# Dedicated Bench Targets (Method B - Selectable directly from IDE status bar)
# ==============================================================================

# 1. Node 1 Pi Shield Bench Target
add_executable(bench_node1_pi_shield.elf)
target_sources(bench_node1_pi_shield.elf PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Src/bsp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Src/can_f4.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../node1_pi_shield/Core/Src/app.c
    ${MX_Application_Src}
)
target_include_directories(bench_node1_pi_shield.elf PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../drivers/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../node1_pi_shield/Core/Inc
)
target_link_options(bench_node1_pi_shield.elf PRIVATE
    -T "${CMAKE_CURRENT_SOURCE_DIR}/STM32F446xx_FLASH.ld"
    -Wl,-Map=bench_node1_pi_shield.map
    -Wl,--gc-sections
)
target_link_libraries(bench_node1_pi_shield.elf
    STM32_Drivers
    stm32cubemx
    rov_shared
    rov_drivers
    m
)

# 2. Node 2 Control Board Bench Target
add_executable(bench_node2_control_board.elf)
target_sources(bench_node2_control_board.elf PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Src/bsp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Src/can_f4.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../node2_control_board/Core/Src/app.c
    ${MX_Application_Src}
)
target_include_directories(bench_node2_control_board.elf PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../drivers/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../node2_control_board/Core/Inc
)
target_link_options(bench_node2_control_board.elf PRIVATE
    -T "${CMAKE_CURRENT_SOURCE_DIR}/STM32F446xx_FLASH.ld"
    -Wl,-Map=bench_node2_control_board.map
    -Wl,--gc-sections
)
target_link_libraries(bench_node2_control_board.elf
    STM32_Drivers
    stm32cubemx
    rov_shared
    rov_drivers
    m
)

# 3. Node 3 Power Slab Bench Target
add_executable(bench_node3_power_slab.elf)
target_sources(bench_node3_power_slab.elf PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Src/bsp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Src/can_f4.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../node3_power_slab/Core/Src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../node3_power_slab/Core/Src/power_sequence.c
    ${MX_Application_Src}
)
target_include_directories(bench_node3_power_slab.elf PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../drivers/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../node3_power_slab/Core/Inc
)
target_link_options(bench_node3_power_slab.elf PRIVATE
    -T "${CMAKE_CURRENT_SOURCE_DIR}/STM32F446xx_FLASH.ld"
    -Wl,-Map=bench_node3_power_slab.map
    -Wl,--gc-sections
)
target_link_libraries(bench_node3_power_slab.elf
    STM32_Drivers
    stm32cubemx
    rov_shared
    rov_drivers
    m
)

# 4. Generic Developer Sandbox / Scratchpad Target (zero app modifications required)
if(NOT DEFINED SANDBOX_FILE OR SANDBOX_FILE STREQUAL "")
    set(SANDBOX_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../../sandbox/sandbox.c")
endif()
get_filename_component(SANDBOX_FILE_ABS "${SANDBOX_FILE}" ABSOLUTE)

add_executable(bench_sandbox.elf)
target_sources(bench_sandbox.elf PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Src/bsp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../Src/can_f4.c
    ${SANDBOX_FILE_ABS}
    ${MX_Application_Src}
)
target_include_directories(bench_sandbox.elf PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../drivers/include
)
target_link_options(bench_sandbox.elf PRIVATE
    -T "${CMAKE_CURRENT_SOURCE_DIR}/STM32F446xx_FLASH.ld"
    -Wl,-Map=bench_sandbox.map
    -Wl,--gc-sections
)
target_link_libraries(bench_sandbox.elf
    STM32_Drivers
    stm32cubemx
    rov_shared
    rov_drivers
    m
)


