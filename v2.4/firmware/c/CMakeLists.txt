set(PROJECT bonobo)
cmake_minimum_required(VERSION 3.12)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(${PROJECT} C CXX ASM)

set(PICO_BOARD pico)
pico_sdk_init()

add_executable(${PROJECT} bonobo.cpp)
pico_generate_pio_header(${PROJECT}  ${CMAKE_CURRENT_LIST_DIR}/control.pio)
pico_generate_pio_header(${PROJECT}  ${CMAKE_CURRENT_LIST_DIR}/status.pio)

target_sources(${PROJECT} PRIVATE bonobo.cpp)
target_link_libraries(${PROJECT} PRIVATE pico_stdlib hardware_pio)
pico_add_extra_outputs(${PROJECT})

pico_enable_stdio_usb(${PROJECT} 1)
pico_enable_stdio_uart(${PROJECT} 0)
