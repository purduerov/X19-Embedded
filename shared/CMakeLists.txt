add_library(rov_shared STATIC
    src/rov_can_protocol.c
    src/rov_pwm_ramp.c
    src/rov_i2c_recovery.c
    src/rov_safety.c
    src/rov_bsp_stub.c
)

add_library(x19_shared ALIAS rov_shared)

target_include_directories(rov_shared PUBLIC
    \/include
)

target_compile_features(rov_shared PUBLIC c_std_11)
