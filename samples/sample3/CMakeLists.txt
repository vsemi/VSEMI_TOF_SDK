cmake_minimum_required(VERSION 3.5.0)

PROJECT(tof_app)

SET(VSEMI_TARGET_PLATFORM "x64_ubuntu_16.04" CACHE STRING "Target platform for Vsemi ToF Driver")

message("   VSEMI_TARGET_PLATFORM set to ${VSEMI_TARGET_PLATFORM}")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(Boost COMPONENTS system REQUIRED)

find_package(OpenCV REQUIRED)

message("OpenCV version: ${OpenCV_VERSION}")

find_package (Threads)

find_package(PCL REQUIRED COMPONENTS common io visualization)

file(GLOB vsemi_tof_driver_INCLUDE_DIR
    "../../driver/include"
)

file(GLOB vsemi_tof_driver_LIB_DIR
    "../../driver/lib/${VSEMI_TARGET_PLATFORM}"
)

file(GLOB vsemi_tof_driver_APP
    "src/*.cpp"
    "src/*.h"
)

include_directories( ${Boost_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${vsemi_tof_driver_INCLUDE_DIR})

link_directories( ${OpenCV_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS} ${vsemi_tof_driver_LIB_DIR})

add_definitions(${PCL_DEFINITIONS})

ADD_EXECUTABLE(vsemi_tof_cam ${vsemi_tof_driver_APP})

TARGET_LINK_LIBRARIES(vsemi_tof_cam ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} vsemi-tof-driver)

set_property(TARGET vsemi_tof_cam PROPERTY CXX_STANDARD 11)
