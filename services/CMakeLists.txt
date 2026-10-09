# ==============================================================================
# Purdue ROV (X19-ROV) - Hardware-Agnostic Domain Services Library
# ==============================================================================

add_library(rov_services STATIC
    src/env_service.c
    src/safety_service.c
    src/actuator_service.c
    src/power_service.c
)

target_include_directories(rov_services PUBLIC
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../shared/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../drivers/include
)

target_link_libraries(rov_services PUBLIC
    rov_shared
    rov_drivers
)

target_compile_features(rov_services PUBLIC c_std_11)
