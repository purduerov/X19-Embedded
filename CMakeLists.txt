cmake_minimum_required(VERSION 3.22)
project(ROV-Embedded C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

# Ensure cached SANDBOX_FILE points to an existing file; fallback to sandbox/sandbox.c if deleted
if(DEFINED SANDBOX_FILE AND NOT EXISTS "${SANDBOX_FILE}")
    set(SANDBOX_FILE "${CMAKE_CURRENT_SOURCE_DIR}/sandbox/sandbox.c" CACHE FILEPATH "Sandbox source file" FORCE)
endif()

option(X19_BUILD_NODE1_ROOT
       "Build the Node 1 STM32C542 image in the root cross build (needs generated HAL/startup; see docs/target-integration-blockers.md)"
       OFF)
option(X19_BUILD_NODE2_ROOT
       "Build Node 2 using the root cross-compile toolchain"
       OFF)
option(X19_BUILD_NODE3_ROOT
       "Build the Node 3 STM32C542 image in the root cross build (needs generated HAL/startup; see docs/target-integration-blockers.md)"
       OFF)

# Strict compiler warnings for subsea embedded safety
add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wdouble-promotion
    -Werror
    $<$<CONFIG:Release>:-O2>
    $<$<CONFIG:Debug>:-Og>
)

# Common include directories available across all targets
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/shared/include
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/include
)

# Add shared libraries
add_subdirectory(shared)
add_subdirectory(drivers)
add_subdirectory(services)

# Add hardware boards and application nodes (only when cross-compiling for embedded targets)
if(CMAKE_CROSSCOMPILING)
    add_subdirectory(boards)
    if(X19_BUILD_NODE1_ROOT)
        add_subdirectory(nodes/node1_pi_shield)
    endif()
    if(X19_BUILD_NODE2_ROOT)
        add_subdirectory(nodes/node2_control_board)
    endif()
    if(X19_BUILD_NODE3_ROOT)
        add_subdirectory(nodes/node3_power_slab)
    endif()
endif()

# Add unit tests (host-only)
if(NOT CMAKE_CROSSCOMPILING)
    add_subdirectory(tests)
endif()
