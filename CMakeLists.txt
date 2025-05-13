cmake_minimum_required(VERSION 3.8)
project(pal_transmissions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_loader_plugins SHARED
  src/half_differential_transmission_loader.cpp
)
target_link_libraries(${PROJECT_NAME}_loader_plugins
    ${EIGEN3_LIBRARIES})

pluginlib_export_plugin_description_file(transmission_interface ros_control_plugins.xml)

ament_auto_package()
