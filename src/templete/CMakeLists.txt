cmake_minimum_required(VERSION 3.17)
project(HC32f460Test C ASM)

#set(CMAKE_C_STANDARD 99)

add_definitions(
        -DHC32F46x
        -DUSE_DEVICE_DRIVER_LIB
#        -D__DEBUG
        #        -D__TARGET_FPU_VFP
        #        -D__FPU_PRESENT=1
        #        -DARM_MATH_CM4
        #        -DARM_MATH_MATRIX_CHECK
        #        -DARM_MATH_ROUNDING
        #        -D__MPU_PRESENT=1
)

file(GLOB_RECURSE USER_SOURCES "source/*.c")
file(GLOB_RECURSE HAL_SOURCES "driver/src/*.c")

add_library(CMSIS
        driver/src/common/system_hc32f46x.c
        startup_hc32f46x.S)

include_directories(mcu/GCC/CMSIS/Core/Include)
include_directories(driver/inc)
include_directories(driver/src/common)

add_executable(${PROJECT_NAME}.elf ${USER_SOURCES} ${HAL_SOURCES} ${LINKER_SCRIPT})

target_link_libraries(${PROJECT_NAME}.elf CMSIS)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.map")
set(HEX_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.bin)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex ${PROJECT_NAME}.elf ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary ${PROJECT_NAME}.elf ${BIN_FILE}
        COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")
