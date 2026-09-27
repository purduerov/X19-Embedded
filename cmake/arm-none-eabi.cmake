set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

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
    find_program(ARM_OBJDUMP arm-none-eabi-objdump HINTS "${ARM_TOOLCHAIN_BIN_DIR}")
    find_program(ARM_SIZE arm-none-eabi-size HINTS "${ARM_TOOLCHAIN_BIN_DIR}")

    set(CMAKE_C_COMPILER "${ARM_GCC_EXECUTABLE}")
    set(CMAKE_ASM_COMPILER "${ARM_GCC_EXECUTABLE}")
    if(ARM_GXX_EXECUTABLE)
        set(CMAKE_CXX_COMPILER "${ARM_GXX_EXECUTABLE}")
    else()
        set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
    endif()
    if(ARM_OBJCOPY)
        set(CMAKE_OBJCOPY "${ARM_OBJCOPY}")
    endif()
    if(ARM_OBJDUMP)
        set(CMAKE_OBJDUMP "${ARM_OBJDUMP}")
    endif()
    if(ARM_SIZE)
        set(CMAKE_SIZE "${ARM_SIZE}")
    endif()
else()
    set(CMAKE_C_COMPILER arm-none-eabi-gcc)
    set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
    set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
    set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
    set(CMAKE_OBJDUMP arm-none-eabi-objdump)
    set(CMAKE_SIZE arm-none-eabi-size)
endif()

# Compiler flags for STM32C542CCT6 (Cortex-M33 with Single-Precision Hardware FPU)
set(COMMON_FLAGS "-mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS} -fdata-sections -ffunction-sections")
set(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS} -fdata-sections -ffunction-sections -fno-exceptions -fno-rtti")
set(CMAKE_ASM_FLAGS_INIT "${COMMON_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${COMMON_FLAGS} -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
