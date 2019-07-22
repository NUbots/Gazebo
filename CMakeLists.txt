CMAKE_MINIMUM_REQUIRED(VERSION 3.10 FATAL_ERROR)

PROJECT(NUbotsGazebo VERSION 1.0.0)

# Find Gazebo and Ignition Transport
FIND_PACKAGE(Protobuf REQUIRED)

# Build the custom protobuf messages Build our builtin protobuf classes
FILE(GLOB_RECURSE protobufs "shared/**.proto")
FOREACH(proto ${protobufs})
    # Get the file without the extension and the directory
    FILE(
        RELATIVE_PATH
        proto
        ${CMAKE_CURRENT_SOURCE_DIR}/shared
        ${proto}
    )
    GET_FILENAME_COMPONENT(file_we ${proto} NAME_WE)
    GET_FILENAME_COMPONENT(dir ${proto} DIRECTORY)

    # Run the protocol buffer compiler on these new protobufs
    ADD_CUSTOM_COMMAND(
        OUTPUT
            "${CMAKE_CURRENT_BINARY_DIR}/shared/${dir}/${file_we}.pb.cc"
            "${CMAKE_CURRENT_BINARY_DIR}/shared/${dir}/${file_we}.pb.h"
        MAIN_DEPENDENCY shared/${proto}
        COMMAND
            ${PROTOBUF_PROTOC_EXECUTABLE}
            ARGS
            --cpp_out=${CMAKE_CURRENT_BINARY_DIR}/shared
            -I${CMAKE_CURRENT_SOURCE_DIR}/shared
            "${CMAKE_CURRENT_SOURCE_DIR}/shared/${dir}/${file_we}.proto"
        COMMENT "Compiling protocol buffer ${proto}"
    )

    LIST(
        APPEND
            msg_src "${CMAKE_CURRENT_BINARY_DIR}/shared/${dir}/${file_we}.pb.cc"
            "${CMAKE_CURRENT_BINARY_DIR}/shared/${dir}/${file_we}.pb.h"
    )
ENDFOREACH(proto)

IF(NOT MSVC)
    # Compilation must be done with c++14 for NUClear to work
    ADD_COMPILE_OPTIONS(-std=c++14 -fPIC)
ENDIF()

ADD_LIBRARY(message OBJECT ${msg_src})
TARGET_COMPILE_FEATURES(message PUBLIC cxx_std_11)
TARGET_INCLUDE_DIRECTORIES(message PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/shared ${PROTOBUF_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(message PUBLIC ${PROTOBUF_LIBRARIES})

# NUClear networking library
FIND_PACKAGE(NUClear REQUIRED)
FIND_PACKAGE(gazebo REQUIRED)

ADD_LIBRARY(nuclear_network OBJECT plugin/nuclear_network.cpp)
TARGET_INCLUDE_DIRECTORIES(nuclear_network PUBLIC ${NUClear_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(nuclear_network PUBLIC ${NUClear_LIBRARIES})
TARGET_INCLUDE_DIRECTORIES(nuclear_network PRIVATE ${GAZEBO_INCLUDE_DIRS})
TARGET_LINK_DIRECTORIES(nuclear_network PRIVATE ${GAZEBO_LIBRARY_DIRS})

# Build our plugins
FILE(GLOB_RECURSE plugins "plugin/**_plugin.cpp")
FOREACH(plugin ${plugins})
    GET_FILENAME_COMPONENT(plugin_name ${plugin} NAME_WE)
    ADD_LIBRARY(${plugin_name} SHARED ${plugin})
    TARGET_INCLUDE_DIRECTORIES(${plugin_name} PRIVATE ${GAZEBO_INCLUDE_DIRS})
    TARGET_LINK_DIRECTORIES(${plugin_name} PRIVATE ${GAZEBO_LIBRARY_DIRS})
    TARGET_LINK_LIBRARIES(${plugin_name} PRIVATE message nuclear_network ${GAZEBO_LIBRARIES})
    LIST(APPEND plugin_targets ${plugin_name})
ENDFOREACH(plugin)

# Install plugin libraries to Gazebo plugin directory
INSTALL(TARGETS ${plugin_targets} LIBRARY DESTINATION ${GAZEBO_PLUGIN_PATH})

# Get a list of all model files to install and install them
FILE(GLOB_RECURSE MODEL_FILES "models/**")
FOREACH(model_file ${MODEL_FILES})
    FILE(
        RELATIVE_PATH
        path
        "${CMAKE_CURRENT_SOURCE_DIR}/models"
        ${model_file}
    )
    INSTALL(FILES ${model_file} DESTINATION "${GAZEBO_MODEL_PATH}/${path}")
ENDFOREACH()
