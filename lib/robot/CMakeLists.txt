add_library(
    robot_lib
    src/Robot.cpp
    include/Robot.h
)

target_link_libraries(robot_lib defines_constants_lib stepper_lib as5600_lib dcmotor_lib jointcontroller_lib EEPROM_lib motorcontroller_lib gripper_lib hardware_pio hardware_irq hardware_i2c gpio_hal)
target_include_directories(robot_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include")