add_library(
    dcmotor_lib
    src/DCMotor.cpp
    include/DCMotor.h
)

target_link_libraries(dcmotor_lib defines_constants_lib gpio_hal pwm_hal)
target_include_directories(dcmotor_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include")