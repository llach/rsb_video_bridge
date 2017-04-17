cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Project Configuration
project("rsb_video_bridge")
set(VERSION_MAJOR  0 CACHE STRING "Major project version part")
set(VERSION_MINOR 15 CACHE STRING "Minor project version part")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslaunch
        sensor_msgs
        cv_bridge
        )


##################
## launch files ##
##################

roslaunch_add_file_check(launch)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        #   INCLUDE_DIRS include
        #  LIBRARIES video_sender
        CATKIN_DEPENDS roscpp sensor_msgs cv_bridge
        #  DEPENDS system_lib
)

###########
## Build ##
###########

# --- RSB dependencies ---

find_package(RSC REQUIRED)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${RSC_CMAKE_MODULE_PATH}")

find_package(RSB REQUIRED)
find_package(RST COMPONENTS sandbox REQUIRED)
find_package(rst-converters REQUIRED)
find_package(ProtocolBuffers REQUIRED)

if(NOT RSC_INTERNAL_BOOST_UUID)
    find_package(BoostUUID REQUIRED)
endif()

find_package(OpenCV)

if(NOT OpenCV_FOUND)
    set(OpenCV_LIBS cv cvaux highgui)
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${catkin_INCLUDE_DIRS}
)

include_directories(BEFORE SYSTEM ${RSC_INCLUDE_DIRS}
        ${RSB_INCLUDE_DIRS}
        ${RST_INCLUDE_DIRS}
        ${RSTSANDBOX_INCLUDE_DIRS}
        ${RST_CONVERTERS_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${BOOSTUUID_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PROTOBUF_INCLUDE_DIR}
)

link_directories(${Boost_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

add_definitions(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})

## Declare a C++ executable
add_executable(video_bridge src/video_bridge.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(video_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(video_bridge
        ${catkin_LIBRARIES}
        ${RSC_LIBRARIES}
        ${RSB_LIBRARIES}
        ${RST_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${PROTOBUF_LIBRARIES}
        ${RST_CONVERTERS_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS video_bridge
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
