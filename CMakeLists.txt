cmake_minimum_required(VERSION 3.12)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(sumo C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
include_directories(include)
add_executable(${PROJECT_NAME}
        core0.c
        core1.c
        drivers/i2c.c
        drivers/pwm.c
        drivers/lsm6dsox.c
)
pico_add_extra_outputs(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
            pico_stdlib
            pico_stdio_usb
            hardware_pwm
            hardware_gpio
            hardware_adc
            hardware_i2c
            hardware_sync
            pico_multicore
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
