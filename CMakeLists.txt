cmake_minimum_required(VERSION 2.8.3)
project(pal_transmissions)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX14)
   message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++14 support.")
   set(CMAKE_CXX_STANDARD 14)
elseif(COMPILER_SUPPORTS_CXX11)
   message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
   set(CMAKE_CXX_STANDARD 11)
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  hardware_interface
  pluginlib
  roscpp
  transmission_interface
)

# Include a custom cmake file for TinyXML
find_package(TinyXML REQUIRED)

# Declare a catkin package
catkin_package(
  LIBRARIES
  INCLUDE_DIRS
    include
  DEPENDS
    pluginlib
    roscpp
    TinyXML
    transmission_interface
)

###########
## Build ##
###########

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_loader_plugins src/head_transmission_loader.cpp
                                           src/half_differential_transmission_loader.cpp)
target_link_libraries(${PROJECT_NAME}_loader_plugins ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_loader_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ros_control_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(resource_retriever)
  include_directories(SYSTEM ${resource_retriever_INCLUDE_DIRS})

  catkin_add_gtest(head_transmission_test test/head_transmission_test.cpp)

  catkin_add_gtest(head_transmission_loader_test test/head_transmission_loader_test.cpp)
  target_link_libraries(head_transmission_loader_test ${catkin_LIBRARIES}
                                                           ${resource_retriever_LIBRARIES})

  add_dependencies(head_transmission_loader_test ${PROJECT_NAME}_loader_plugins)
endif()
