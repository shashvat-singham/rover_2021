project(obstacle_avoidance)
cmake_minimum_required(VERSION 2.8)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MODULE_PATH /home/tanmay/GeographicLib-1.50.1/cmake)
find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

cs_add_executable(velcontroller src/velocitycontrol.cpp)
cs_add_executable(reachdest src/reachdest.cpp)
cs_add_executable(avoidance_node src/avoidance_node.cpp)


cs_install()
cs_export()

