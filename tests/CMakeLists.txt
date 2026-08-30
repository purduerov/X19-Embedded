enable_testing()

add_executable(test_can_protocol test_can_protocol.c)
target_link_libraries(test_can_protocol PRIVATE x19_shared)
add_test(NAME test_can_protocol COMMAND test_can_protocol)

add_executable(test_pwm_ramp test_pwm_ramp.c)
target_link_libraries(test_pwm_ramp PRIVATE x19_shared)
add_test(NAME test_pwm_ramp COMMAND test_pwm_ramp)

add_executable(test_safety test_safety.c)
target_link_libraries(test_safety PRIVATE x19_shared)
add_test(NAME test_safety COMMAND test_safety)

add_executable(test_i2c_recovery test_i2c_recovery.c)
target_link_libraries(test_i2c_recovery PRIVATE x19_shared)
add_test(NAME test_i2c_recovery COMMAND test_i2c_recovery)
