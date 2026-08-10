add_library(
    continuousservo_lib
    src/ContinuousServo.cpp
    include/ContinuousServo.h
)

target_link_libraries(continuousservo_lib pico_servo_lib pico_stdlib)
target_include_directories(continuousservo_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include")
