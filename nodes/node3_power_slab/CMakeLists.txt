add_executable(node3_power_slab.elf
    Core/Src/main.c
    Core/Src/app.c
)

target_include_directories(node3_power_slab.elf PRIVATE
    Core/Inc
    ../../shared/include
    ../../drivers/include
    "../Shared Library/include"
    "../Hardware Drivers/include"
)

target_link_libraries(node3_power_slab.elf PRIVATE
    rov_shared
    rov_drivers
)

target_compile_features(node3_power_slab.elf PUBLIC c_std_11)