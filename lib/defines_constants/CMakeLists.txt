add_library(
    defines_constants_lib
    include/defines_constants.h
)

set_target_properties(defines_constants_lib PROPERTIES LINKER_LANGUAGE C)
target_include_directories(defines_constants_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")