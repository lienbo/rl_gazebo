cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(CarinaPlugin)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
)
include_directories( ${catkin_INCLUDE_DIRS} )


find_package(gazebo REQUIRED)
include_directories( ${GAZEBO_INCLUDE_DIRS} )
link_directories( ${GAZEBO_LIBRARY_DIRS} )
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(RosRoverPlugin SHARED src/RosRoverPlugin.cpp src/RoverModel.cpp)
target_link_libraries(RosRoverPlugin
    ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)


### Install plugin and models
install(TARGETS RosRoverPlugin
    LIBRARY DESTINATION ${GAZEBO_PLUGIN_PATH})

install(DIRECTORY models/
    DESTINATION $ENV{HOME}/.gazebo/models)
