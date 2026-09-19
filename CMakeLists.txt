# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(ffb C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(ffb
    main.c
    hid/ffb_hid.c
    hid/gamepad_hid.c
    motor/motor_control.c
    motor/encoder_uart.c
    motor/mcp3204.c
    usb_descriptors.c
    )

pico_set_program_name(ffb "ffb")
pico_set_program_version(ffb "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(ffb 1)
pico_enable_stdio_usb(ffb 0)

# Add the standard library to the build
target_link_libraries(ffb
        pico_stdlib
        tinyusb_device
        tinyusb_board
        hardware_gpio
        hardware_pwm
        hardware_adc
        hardware_clocks
        hardware_uart
        hardware_sync
        hardware_spi
        hardware_irq
        hardware_dma
        pico_multicore
        pico_time
        )

# Add the standard include files to the build
target_include_directories(ffb PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(ffb)
