if(NOT DEFINED MAIN_SOURCE)
    message(FATAL_ERROR "MAIN_SOURCE was not provided")
endif()

file(READ "${MAIN_SOURCE}" MAIN_CONTENTS)

# Strip C comments before searching. Without this, a startup call that has been
# commented out still satisfies the check below, so a node whose main.c no longer
# calls HAL_Init() or the peripheral MX_* initialisers would pass anyway. The
# ordering contract is about executed code, not about text present in the file.
string(REGEX REPLACE "/\\*[^*]*\\*+([^/*][^*]*\\*+)*/" " " MAIN_CONTENTS "${MAIN_CONTENTS}")
string(REGEX REPLACE "//[^\n]*" " " MAIN_CONTENTS "${MAIN_CONTENTS}")

set(EXPECTED_CALLS
    "HAL_Init()"
    "SystemClock_Config()"
    "MX_GPIO_Init()"
    "MX_FDCAN1_Init()"
    "MX_I2C1_Init()"
    "MX_TIM1_Init()"
    "MX_TIM8_Init()"
    "app_main()"
)

set(PREVIOUS_POSITION -1)

foreach(CALL IN LISTS EXPECTED_CALLS)
    string(FIND "${MAIN_CONTENTS}" "${CALL}" POSITION)

    if(POSITION EQUAL -1)
        message(FATAL_ERROR "Missing startup call: ${CALL}")
    endif()

    if(NOT PREVIOUS_POSITION EQUAL -1 AND POSITION LESS PREVIOUS_POSITION)
        message(FATAL_ERROR "Startup call is out of order: ${CALL}")
    endif()

    set(PREVIOUS_POSITION ${POSITION})
endforeach()

message(STATUS "Node 2 generated startup order passed")