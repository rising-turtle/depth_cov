cmake_minimum_required(VERSION 2.8.3)
project(depth_cov)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_filters
  sensor_msgs
  cv_bridge
  rosbag
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES img_pub
   CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

# CXX flags 
set(CMAKE_CXX_FLAGS "-ggdb -g -O3 -w -std=c++11 -fpermissive") # -fPIC -fpermissive

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ./src
  "/usr/include/eigen3"
)
####### OpenCV #######
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

###########
## Build ##
###########

add_executable(depth_cov src/depth_cov.cpp)
target_link_libraries(depth_cov ${catkin_LIBRARIES} ${OpenCV_LIBS})
