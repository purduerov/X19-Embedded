set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# Auto-detect arm-none-eabi-gcc from PATH or standard install paths
find_program(ARM_GCC_EXECUTABLE arm-none-eabi-gcc
    PATHS
        "C:/ST/STM32CubeCLT_1.22.0/GNU-tools-for-STM32/bin"
        "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/14.2 rel1/bin"
        "C:/ST/STM32CubeCLT/GNU-tools-for-STM32/bin"
        "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/*/bin"
        "C:/Program Files/Arm GNU Toolchain arm-none-eabi/*/bin"
        "C:/Program Files (x86)/GNU Arm Embedded Toolchain/*/bin"
        "C:/Program Files/GNU Arm Embedded Toolchain/*/bin"
        "/Applications/ArmGNUToolchain/*/arm-none-eabi/bin"
        "/opt/st/stm32cubeclt*/GNU-tools-for-STM32/bin"
        "/opt/arm-none-eabi/bin"
        "/usr/bin"
)

if(ARM_GCC_EXECUTABLE)
    get_filename_component(ARM_TOOLCHAIN_BIN_DIR "${ARM_GCC_EXECUTABLE}" DIRECTORY)
    find_program(ARM_GXX_EXECUTABLE arm-none-eabi-g++ HINTS "${ARM_TOOLCHAIN_BIN_DIR}")
    find_program(ARM_OBJCOPY arm-none-eabi-objcopy HINTS "${ARM_TOOLCHAIN_BIN_DIR}")
    find_program(ARM_SIZE arm-none-eabi-size HINTS "${ARM_TOOLCHAIN_BIN_DIR}")

    set(CMAKE_C_COMPILER "${ARM_GCC_EXECUTABLE}")
    set(CMAKE_ASM_COMPILER "${ARM_GCC_EXECUTABLE}")
    if(ARM_GXX_EXECUTABLE)
        set(CMAKE_CXX_COMPILER "${ARM_GXX_EXECUTABLE}")
        set(CMAKE_LINKER "${ARM_GXX_EXECUTABLE}")
    else()
        set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
        set(CMAKE_LINKER arm-none-eabi-g++)
    endif()
    if(ARM_OBJCOPY)
        set(CMAKE_OBJCOPY "${ARM_OBJCOPY}")
    endif()
    if(ARM_SIZE)
        set(CMAKE_SIZE "${ARM_SIZE}")
    endif()
else()
    set(TOOLCHAIN_PREFIX                arm-none-eabi-)
    set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
    set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
    set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)
    set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++)
    set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
    set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)
endif()

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F446xx_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
