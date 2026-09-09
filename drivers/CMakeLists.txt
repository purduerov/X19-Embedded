add_library(rov_drivers STATIC
    src/tcan1044.c
    src/lsm6dsoxtr.c
    src/bmi270.c
    src/ms5837.c
    src/bme280.c
    src/ina226.c
    src/ina237.c
    src/tmp1075.c
    src/tps25990.c
    src/pmbus_brick.c
)

add_library(x19_drivers ALIAS rov_drivers)

target_include_directories(rov_drivers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/include
)

target_compile_features(rov_drivers PUBLIC c_std_11)
