add_executable(node2_control_board.elf
    Core/Src/main.c
)

target_include_directories(node2_control_board.elf PRIVATE
    Core/Inc
    ../../shared/include
    ../../drivers/include
)

target_link_libraries(node2_control_board.elf PRIVATE
    x19_shared
    x19_drivers
)

target_compile_features(node2_control_board.elf PUBLIC c_std_11)
