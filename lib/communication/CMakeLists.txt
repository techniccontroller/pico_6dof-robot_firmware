add_library(
    communication_lib
    src/Communication.cpp
    include/Communication.h
)

target_link_libraries(communication_lib defines_constants_lib robot_lib comm_hal)
target_include_directories(communication_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include")