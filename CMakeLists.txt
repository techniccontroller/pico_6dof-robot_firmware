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
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_6dof-robot_main C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(hal)
add_subdirectory(lib)


# Add executable. Default name is the project name, version 0.1
add_executable(pico_6dof-robot_main pico_6dof-robot_main.cpp )

pico_set_program_name(pico_6dof-robot_main "pico_6dof-robot_main")
pico_set_program_version(pico_6dof-robot_main "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pico_6dof-robot_main 0)
pico_enable_stdio_usb(pico_6dof-robot_main 1)

target_compile_definitions(pico_6dof-robot_main PRIVATE
        PICO_MAX_SHARED_IRQ_HANDLERS=5
)

target_compile_options(pico_6dof-robot_main PRIVATE -Wno-psabi)

# Add the standard library to the build
target_link_libraries(pico_6dof-robot_main
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(pico_6dof-robot_main PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(pico_6dof-robot_main
        hardware_i2c
        hardware_pio
        hardware_irq
        hardware_clocks
        comm_hal
        gpio_hal
        dcmotor_lib
        stepper_lib
        communication_lib
        as5600_lib
        EEPROM_lib
        robot_lib
        pico_cyw43_arch_none # we need Wifi to access the GPIO, but we don't need anything else
        )

pico_add_extra_outputs(pico_6dof-robot_main)


