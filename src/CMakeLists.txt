
add_library("${LIB_NAME}" SHARED
  Planner.cpp
  "${path_planner_SOURCE_DIR}/include/planner/Planner.hpp"
  # "${path_planner_SOURCE_DIR}/include/spline.h"
  # "${path_planner_SOURCE_DIR}/include/helpers.h"
)

# We need this directory, and users of our library will need it too
# target_include_directories("${LIB_NAME}" PRIVATE ../include)
target_include_directories("${LIB_NAME}" PUBLIC ../include)

# This depends on (header only) eigen
target_link_libraries("${LIB_NAME}" PRIVATE nlohmann_json::nlohmann_json Eigen3::Eigen)
target_compile_features("${LIB_NAME}" PUBLIC cxx_std_11)