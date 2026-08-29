add_library(x19_drivers STATIC
    src/tcan1044.c
    src/lsm6dsoxtr.c
    src/ms5837.c
    src/bme280.c
    src/ina226.c
    src/tps25990.c
)

target_include_directories(x19_drivers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/include
)

target_compile_features(x19_drivers PUBLIC c_std_11)
