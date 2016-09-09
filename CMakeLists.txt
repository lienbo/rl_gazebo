cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(CarinaPlugin)


### Compile the protobuf message
find_package(Protobuf REQUIRED)
if (NOT PROTOBUF_FOUND)
  BUILD_ERROR ("Missing: Google Protobuf (libprotobuf-dev)")
endif()
include_directories( ${PROTOBUF_INCLUDE_DIRS} )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS src/carina_plugin.proto)


### Compile the Gazebo plugin
find_package(gazebo REQUIRED)

include_directories( ${GAZEBO_INCLUDE_DIRS} )
link_directories( ${GAZEBO_LIBRARY_DIRS} )
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(CarinaPlugin SHARED
    src/CarinaPlugin.cpp
    ${PROTO_SRCS}
    ${PROTO_HDRS})

target_link_libraries(CarinaPlugin
    ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PROTOBUF_LIBRARIES})
