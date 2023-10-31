add_library(
    motorcontroller_lib
    src/MotorController.cpp
    include/MotorController.h
)

target_link_libraries(motorcontroller_lib defines_constants_lib stepper_lib as5600_lib dcmotor_lib)
target_include_directories(motorcontroller_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include")