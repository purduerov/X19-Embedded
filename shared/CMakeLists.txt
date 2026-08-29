add_library(x19_shared STATIC
    src/x19_can_protocol.c
    src/x19_pwm_ramp.c
    src/x19_i2c_recovery.c
    src/x19_safety.c
)

target_include_directories(x19_shared PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(x19_shared PUBLIC c_std_11)
