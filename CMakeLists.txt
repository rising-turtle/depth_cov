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

####### Ceres #######
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})


###########
## Build ##
###########

add_executable(depth_cov src/depth_cov.cpp)
target_link_libraries(depth_cov ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(grid_point_std src/grid_point_std.cpp)
target_link_libraries(grid_point_std ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(gmm_depth src/gmm_depth.cpp)
target_link_libraries(gmm_depth ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(gmm_inv_depth src/gmm_inv_depth.cpp)
target_link_libraries(gmm_inv_depth ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(pt_distribute src/pt_distribute.cpp)
target_link_libraries(pt_distribute ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(depth_distribute src/depth_distribute.cpp)
target_link_libraries(depth_distribute ${catkin_LIBRARIES} ${OpenCV_LIBS})


###########
## Ceres ##
###########

add_executable(test_ceres src/test_ceres.cpp)
target_link_libraries(test_ceres ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_executable(solve_lambda src/solve_lambda.cpp)
target_link_libraries(solve_lambda ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_executable(solve_lambda_folder src/solve_lambda_folder.cpp)
target_link_libraries(solve_lambda_folder ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_executable(test_std src/test_std.cpp)
target_link_libraries(test_std ${catkin_LIBRARIES} ${CERES_LIBRARIES})