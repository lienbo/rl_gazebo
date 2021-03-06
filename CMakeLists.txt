cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(DRLPlugin)


find_package(gazebo REQUIRED)
include_directories( ${GAZEBO_INCLUDE_DIRS} )
link_directories( ${GAZEBO_LIBRARY_DIRS} )
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(Boost REQUIRED COMPONENTS system filesystem )
include_directories( ${Boost_INCLUDE_DIRS} )
link_directories( ${Boost_LIBRARY_DIRS} )

add_library(RoverPlugin SHARED
    src/RoverPlugin.cpp
    src/RoverModel.cpp
    src/QLearner.cpp
    src/State.cpp)

target_link_libraries(RoverPlugin
    ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES})

### Install plugin and models
install(TARGETS RoverPlugin
    LIBRARY DESTINATION ${GAZEBO_PLUGIN_PATH})

install(DIRECTORY gazebo/models/
    DESTINATION $ENV{HOME}/.gazebo/models)


### Optional - Compile and install ROS plugin
if( COMPILE_ROS_PLUGIN )
    find_package(catkin COMPONENTS
        roscpp
        std_msgs
        message_generation)

    include_directories( ${catkin_INCLUDE_DIRS} )

    add_library(RosRoverPlugin SHARED src/RosRoverPlugin.cpp src/RoverModel.cpp)

    target_link_libraries(RosRoverPlugin
        ${GAZEBO_LIBRARIES}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES})

    install(TARGETS RosRoverPlugin
        LIBRARY DESTINATION ${GAZEBO_PLUGIN_PATH})
endif()


### Compile and install deep reinforcement learning plugin
if( COMPILE_DRL_PLUGIN )
    find_package( Caffe REQUIRED )
    include_directories( ${Caffe_INCLUDE_DIRS} )

    find_package( OpenCV REQUIRED )
    include_directories( ${OpenCV_INCLUDE_DIRS} )

    pkg_check_modules( hdf5 REQUIRED )
    include_directories( ${hdf5_INCLUDE_DIRS} )

    add_library( NFQPlugin SHARED
        src/NFQPlugin.cpp
        src/CaffeRL.cpp
        src/RoverModel.cpp
        src/QLearner.cpp
        src/State.cpp)

    target_link_libraries( NFQPlugin
        ${Caffe_LIBRARIES}
        ${Boost_LIBRARIES}
        glog
        protobuf
        ${hdf5_LIBRARIES}
        ${GAZEBO_LIBRARIES})

    install(TARGETS NFQPlugin
        LIBRARY DESTINATION ${GAZEBO_PLUGIN_PATH})

    add_library( DRLPlugin SHARED
        src/DRLPlugin.cpp
        src/CaffeRL.cpp
        src/RoverModel.cpp
        src/QLearner.cpp
        src/State.cpp)

    target_link_libraries( DRLPlugin
        ${Caffe_LIBRARIES}
        ${Boost_LIBRARIES}
        glog
        protobuf
        ${hdf5_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${GAZEBO_LIBRARIES})

    install(TARGETS DRLPlugin
        LIBRARY DESTINATION ${GAZEBO_PLUGIN_PATH})
endif()
