cmake_minimum_required(VERSION 3.1.2)
project(uvdar_fpga)

FILE(GLOB_RECURSE HEADER_FILES include *.h)
FILE(GLOB_RECURSE LAUNCHER_FILES include *.launch)
FILE(GLOB_RECURSE YAML_FILES include *.yaml)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -std=c++17 -g -Wall -Wextra -O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  cv_bridge
  sensor_msgs
  mrs_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs image_transport cv_bridge sensor_msgs mrs_msgs
)

add_library(devmem include/devmem/devmem.c)
add_library(fpgaint_poll include/fpgaint_poll/fpgaint_poll.c)
add_library(i2c include/i2c/i2c.c)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(camera_node src/camera_node.cpp)

target_link_libraries(camera_node ${catkin_LIBRARIES} devmem fpgaint_poll i2c)

install(TARGETS camera_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


