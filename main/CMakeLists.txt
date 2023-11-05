# Add executable. Default name is the project name, version 0.1
add_executable(pico_stepper-control pico_stepper-control.cpp )

pico_set_program_name(pico_stepper-control "pico_stepper-control")
pico_set_program_version(pico_stepper-control "0.1")

pico_enable_stdio_uart(pico_stepper-control 0)
pico_enable_stdio_usb(pico_stepper-control 1)

target_compile_definitions(pico_stepper-control PRIVATE
        PICO_MAX_SHARED_IRQ_HANDLERS=5
)

# Add the standard library to the build
target_link_libraries(pico_stepper-control 
        pico_stdlib 
        pico_cyw43_arch_none     # we need Wifi to access the GPIO, but we don't need anything else
        )

# Add any user requested libraries
target_link_libraries(pico_stepper-control
        hardware_i2c
        hardware_pio
        hardware_irq
        comm_hal
        gpio_hal
        dcmotor_lib
        stepper_lib
        communication_lib
        as5600_lib
        EEPROM_lib
        robot_lib
        )

pico_add_extra_outputs(pico_stepper-control)
