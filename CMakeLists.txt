cmake_minimum_required(VERSION 3.22)
project(X19-Embedded C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Strict warnings for subsea embedded safety
add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wdouble-promotion
    -Werror
    -O2
)

# Common include directories available across all targets
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/shared/include
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/include
)

# Add shared libraries
add_subdirectory(shared)
add_subdirectory(drivers)

# Subdirectories for application nodes
# add_subdirectory(nodes/node1_pi_shield)
# add_subdirectory(nodes/node2_control_board)
# add_subdirectory(nodes/node3_power_slab)
# add_subdirectory(nodes/node4_usb_hub)
# add_subdirectory(bootloader/stm32_can_bootloader)
