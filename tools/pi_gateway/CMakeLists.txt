add_executable(pi_gateway
    main.cpp
)

target_include_directories(pi_gateway PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/include
)

target_link_libraries(pi_gateway PRIVATE
    rov_shared
)

if(WIN32)
    target_link_libraries(pi_gateway PRIVATE ws2_32)
    if(MINGW)
        target_link_options(pi_gateway PRIVATE -static)
    endif()
endif()

if(NOT WIN32 OR MINGW)
    target_link_libraries(pi_gateway PRIVATE m)
endif()
