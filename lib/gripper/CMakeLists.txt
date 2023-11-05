add_library(
    gripper_lib
    src/Gripper.cpp
    include/Gripper.h
)

target_link_libraries(gripper_lib defines_constants_lib pico_servo_lib)
target_include_directories(gripper_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include")