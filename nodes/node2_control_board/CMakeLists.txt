cmake_minimum_required(VERSION 3.22)

# Standalone STM32G474 Nucleo bench build
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)

    project(node2_control_board C ASM)

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_C_STANDARD_REQUIRED ON)

    # CubeMX generated target
    add_executable(node2_control_board)

    # Add generated HAL/startup sources, includes, and definitions
    add_subdirectory(cmake/stm32cubemx)

    # Node 2 application layer
    target_sources(node2_control_board PRIVATE
        Core/Src/app.c
        Core/Src/bsp.c
    )

    target_include_directories(node2_control_board PRIVATE
        Core/Inc
        ../../shared/include
        ../../drivers/include
    )

    # Existing project libraries
    add_subdirectory(../../shared ${CMAKE_BINARY_DIR}/shared)
    add_subdirectory(../../drivers ${CMAKE_BINARY_DIR}/drivers)

    target_link_libraries(node2_control_board
        rov_shared
        rov_drivers
        m
    )

    target_compile_options(node2_control_board PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wdouble-promotion
        -Werror
    )

    # Standalone I2C Bus Scanner utility (zero app modifications required)
    add_executable(node2_i2c_scanner)
    target_sources(node2_i2c_scanner PRIVATE
        ${MX_Application_Src}
        Core/Src/diagnostic_i2c_scan.c
        Core/Src/bsp.c
    )
    target_include_directories(node2_i2c_scanner PRIVATE
        Core/Inc
        ../../shared/include
        ../../drivers/include
    )
    target_link_libraries(node2_i2c_scanner
        STM32_Drivers
        rov_shared
        rov_drivers
        m
    )
    target_compile_options(node2_i2c_scanner PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wdouble-promotion
        -Werror
    )

    # Generic Developer Sandbox / Scratchpad Target (zero app modifications required)
    if(NOT DEFINED SANDBOX_FILE OR SANDBOX_FILE STREQUAL "")
        set(SANDBOX_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../sandbox/sandbox.c")
    endif()
    get_filename_component(SANDBOX_FILE_ABS "${SANDBOX_FILE}" ABSOLUTE)

    add_executable(sandbox)
    target_sources(sandbox PRIVATE
        ${MX_Application_Src}
        ${SANDBOX_FILE_ABS}
        Core/Src/bsp.c
    )
    target_include_directories(sandbox PRIVATE
        Core/Inc
        ../../shared/include
        ../../drivers/include
    )
    target_link_libraries(sandbox
        STM32_Drivers
        rov_shared
        rov_drivers
        m
    )
    target_compile_options(sandbox PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wdouble-promotion
        -Werror
    )

else()

    # Existing monorepo target
    add_executable(node2_control_board.elf
        Core/Src/main.c
        Core/Src/app.c
        Core/Src/bsp.c
    )

    target_include_directories(node2_control_board.elf PRIVATE
        Core/Inc
        ../../shared/include
        ../../drivers/include
        "../Shared Library/include"
        "../Hardware Drivers/include"
    )

    target_link_libraries(node2_control_board.elf PRIVATE
        rov_shared
        rov_drivers
        m
    )

    target_compile_features(node2_control_board.elf PUBLIC c_std_11)

endif()