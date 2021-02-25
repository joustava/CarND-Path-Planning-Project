add_library("${LIB_NAME}"
  # "${CMAKE_SOURCE_DIR}/include/helpers.hpp"
)

# This depends on (header only) eigen
target_link_libraries("${LIB_NAME}" Eigen3::Eigen)
target_include_directories("${LIB_NAME}" PUBLIC ../include)
target_compile_features("${LIB_NAME}" PUBLIC cxx_std_11)