add_library(
    communication_lib
    src/communication.c
    include/communication.h
)

target_link_libraries(communication_lib defines_constants_lib motor_lib comm_hal)
target_include_directories(communication_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include")