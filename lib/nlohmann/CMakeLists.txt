add_library(
    nlohmann_json_lib
    include/json.hpp
)

set_target_properties(nlohmann_json_lib PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(nlohmann_json_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")