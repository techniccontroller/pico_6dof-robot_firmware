
add_library(
    comm_hal
    src/comm_functions.c
    include/comm_functions.h
)

target_link_libraries(comm_hal pico_stdlib)
target_include_directories(comm_hal PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(
    gpio_hal
    src/gpio_functions.c
    include/gpio_functions.h
)

target_link_libraries(gpio_hal pico_stdlib)
target_include_directories(gpio_hal PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(
    pwm_hal
    src/pwm_functions.c
    include/pwm_functions.h
)

target_link_libraries(pwm_hal pico_stdlib hardware_pwm)
target_include_directories(pwm_hal PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}/include")