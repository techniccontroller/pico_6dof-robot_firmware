add_library(
    stepper_lib
    src/AccelStepper.cpp
    src/MultiStepper.cpp
    src/StepperConfiguration.cpp
    include/AccelStepper.h
    include/MultiStepper.h
    include/StepperConfiguration.h
)

target_link_libraries(stepper_lib defines_constants_lib gpio_hal pwm_hal)
target_include_directories(stepper_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include")