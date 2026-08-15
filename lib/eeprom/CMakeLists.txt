add_library(
    EEPROM_lib
    src/EEPROM.cpp
    include/EEPROM.h
)

target_link_libraries(EEPROM_lib pico_stdlib pico_multicore hardware_flash)
target_include_directories(EEPROM_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include")
