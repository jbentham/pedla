# PEDLA remote logic analyser project

cmake_minimum_required(VERSION 3.12)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

# Console output on UART1
add_definitions(-DPICO_DEFAULT_UART=1)

project(pedla C CXX ASM)

# Set to 0 for Pico-W CYW43439, 1 for Murata 1DX (CYW4343W)
# It may be necessary to reduce PIO_SPI_FREQ when using an external device
set (CHIP_4343W 0)

# Firmware file for CYW43439 or CYW4343W
if (${CHIP_4343W})
    message (STATUS "Building CYW4343W version")
    set (FW_FILE firmware/fw_4343w.c)
    add_compile_options(-DCHIP_4343W)
else ()
    message (STATUS "Building Pico-W CYW43439 version")
    set (FW_FILE firmware_new/fw_43439.c)
endif ()

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME}
    pedla.c mongoose.c la_wifi.c
    picowi/picowi_event.c picowi/picowi_init.c picowi/picowi_join.c
    picowi/picowi_pico.c picowi/picowi_pio.c picowi/picowi_wifi.c
    picowi/picowi_ioctl.c ${FW_FILE} la_pico.c
    la_console.c)

target_compile_options(pedla PRIVATE -Wall -Wextra -Wno-unused-parameter)
# Enable compiler warnings
add_compile_options(-Wall -Wextra)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Create PIO header file
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/picowi/picowi_pio.pio)

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib hardware_spi pico_rand hardware_pio 
    hardware_dma hardware_i2c hardware_pwm )

# Mongoose build flags
add_definitions(-DMG_ENABLE_TCPIP=1)
add_definitions(-DMG_ENABLE_PACKED_FS=0)
add_definitions(-DMG_ENABLE_MBEDTLS=0)
add_definitions(-DMG_ENABLE_CUSTOM_RANDOM=0)
add_definitions(-DMG_ENABLE_FILE=0)

# Enable uart output, disable usb output
pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

# EOF