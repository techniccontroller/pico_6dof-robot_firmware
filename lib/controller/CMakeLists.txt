add_library(
    controller_lib
    src/Controller.cpp
    include/Controller.h
)

target_link_libraries(controller_lib defines_constants_lib stepper_lib as5600_lib dcmotor_lib)
target_include_directories(controller_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include")