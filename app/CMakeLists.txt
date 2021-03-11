add_executable("${PROJECT_NAME}" main.cpp)
target_link_libraries("${PROJECT_NAME}" PRIVATE "${LIB_NAME}" z ssl uv uWS nlohmann_json::nlohmann_json Eigen3::Eigen)
