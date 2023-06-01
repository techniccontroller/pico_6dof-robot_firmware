add_library(
    motor_lib
    src/AccelStepper.c
    src/MultiStepper.c
    include/AccelStepper.h
    include/MultiStepper.h
)

target_link_libraries(motor_lib defines_constants_lib gpio_hal pwm_hal)
target_include_directories(motor_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include")