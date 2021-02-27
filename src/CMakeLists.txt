
add_library("${LIB_NAME}"
  Planner.cpp
)

# This depends on (header only) eigen
target_link_libraries("${LIB_NAME}" PRIVATE Eigen3::Eigen)
target_include_directories("${LIB_NAME}" PUBLIC ../include)
target_compile_features("${LIB_NAME}" PUBLIC cxx_std_11)