add_library(
    as5600_lib
    src/AS5600.cpp
    include/AS5600.h
)

target_link_libraries(as5600_lib hardware_i2c)
target_include_directories(as5600_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include")