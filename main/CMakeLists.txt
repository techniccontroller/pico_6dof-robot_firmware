# Add executable. Default name is the project name, version 0.1
add_executable(pico_stepper-control pico_stepper-control.c )

pico_set_program_name(pico_stepper-control "pico_stepper-control")
pico_set_program_version(pico_stepper-control "0.1")

pico_enable_stdio_uart(pico_stepper-control 0)
pico_enable_stdio_usb(pico_stepper-control 1)

# Add the standard library to the build
target_link_libraries(pico_stepper-control pico_stdlib)

# Add any user requested libraries
target_link_libraries(pico_stepper-control
        hardware_i2c
        hardware_pio
        hardware_irq
        comm_hal
        gpio_hal

        motor_lib
        communication_lib
        as5600_lib
        )

pico_add_extra_outputs(pico_stepper-control)
