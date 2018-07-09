cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
FIND_PACKAGE(ignition-transport4 REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(nubotsigus_plugin SHARED nubotsigus_plugin.cc)
target_link_libraries(nubotsigus_plugin ${GAZEBO_libraries} ${IGNITION-TRANSPORT_LIBRARIES})