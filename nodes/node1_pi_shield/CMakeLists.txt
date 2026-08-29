add_executable(node1_pi_shield.elf
    Core/Src/main.c
)

target_include_directories(node1_pi_shield.elf PRIVATE
    Core/Inc
    ../../shared/include
    ../../drivers/include
)

target_link_libraries(node1_pi_shield.elf PRIVATE
    x19_shared
    x19_drivers
)

target_compile_features(node1_pi_shield.elf PUBLIC c_std_11)
