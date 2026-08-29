add_executable(node3_power_slab.elf
    Core/Src/main.c
)

target_include_directories(node3_power_slab.elf PRIVATE
    Core/Inc
    ../../shared/include
    ../../drivers/include
)

target_link_libraries(node3_power_slab.elf PRIVATE
    x19_shared
    x19_drivers
)

target_compile_features(node3_power_slab.elf PUBLIC c_std_11)
