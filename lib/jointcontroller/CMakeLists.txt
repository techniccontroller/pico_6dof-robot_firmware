add_library(
    jointcontroller_lib
    src/JointController.cpp
    include/JointController.h
)

target_link_libraries(jointcontroller_lib defines_constants_lib stepper_lib as5600_lib dcmotor_lib continuousservo_lib)
target_include_directories(jointcontroller_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include")
