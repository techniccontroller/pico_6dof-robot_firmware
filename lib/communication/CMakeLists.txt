add_library(
    communication_lib
    src/communication.cpp
    include/communication.h
    include/robot_command.h
)

target_link_libraries(communication_lib defines_constants_lib comm_hal pico_util)
target_include_directories(communication_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include")
