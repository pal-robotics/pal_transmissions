cmake_minimum_required(VERSION 3.8)
project(pal_transmissions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_loader_plugins SHARED
  src/half_differential_transmission_loader.cpp
)

pluginlib_export_plugin_description_file(transmission_interface ros_control_plugins.xml)


if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_half_differential_transmission
    test/half_differential_transmission_test.cpp
  )
  target_link_libraries(test_half_differential_transmission ${PROJECT_NAME}_loader_plugins)

  ament_add_gmock(test_half_differential_transmission_loader
    test/half_differential_transmission_loader_test.cpp
  )
  target_link_libraries(test_half_differential_transmission_loader ${PROJECT_NAME}_loader_plugins)
  ament_target_dependencies(test_half_differential_transmission_loader ros2_control_test_assets)
endif()


ament_auto_package()
