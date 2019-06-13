CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)

# Find Gazebo and Ignition Transport
FIND_PACKAGE(gazebo REQUIRED)
FIND_PACKAGE(ignition-transport4 REQUIRED)
INCLUDE_DIRECTORIES(${GAZEBO_INCLUDE_DIRS} ${IGNITION-TRANSPORT_INCLUDE_DIRS})
LINK_DIRECTORIES(${GAZEBO_LIBRARY_DIRS})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugins
ADD_LIBRARY(nubotsigus_plugin SHARED nubotsigus_plugin.cc)
TARGET_LINK_LIBRARIES(nubotsigus_plugin ${GAZEBO_libraries} ${IGNITION-TRANSPORT_LIBRARIES})

ADD_LIBRARY(nubotsworld_plugin SHARED nubotsworld_plugin.cc)
TARGET_LINK_LIBRARIES(nubotsworld_plugin ${GAZEBO_libraries} ${IGNITION-TRANSPORT_LIBRARIES})

ADD_LIBRARY(nubotsball_plugin SHARED nubotsball_plugin.cc)
TARGET_LINK_LIBRARIES(nubotsball_plugin ${GAZEBO_libraries} ${IGNITION-TRANSPORT_LIBRARIES})

# Install plugin libraries to Gazebo plugin directory
INSTALL(TARGETS nubotsigus_plugin nubotsworld_plugin nubotsball_plugin
  LIBRARY DESTINATION ${GAZEBO_PLUGIN_PATH}
)

# Get a list of all model files to install and install them
FILE(GLOB_RECURSE MODEL_FILES "${CMAKE_SOURCE_DIR}/../models/*")
FOREACH(model_file ${MODEL_FILES})
  FILE(RELATIVE_PATH 
       path 
       "${CMAKE_SOURCE_DIR}/../models" 
       ${model_file}
  )
  MESSAGE(STATUS "Model file: ${path}")
  INSTALL(FILES ${model_file} DESTINATION "${GAZEBO_MODEL_PATH}/${path}")
ENDFOREACH()


