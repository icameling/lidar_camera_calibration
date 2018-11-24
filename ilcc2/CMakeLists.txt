cmake_minimum_required(VERSION 2.8.3)
project(ilcc2)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  cv_bridge
  camera_model
  rosbag
  roslib
)

find_package(Eigen3)
include_directories( ${EIGEN3_INCLUDE_DIR} )

#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Ceres
find_package(Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS})
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

# Sophus
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ilcc2_lib
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(ilcc_lib2
    src/ImageCornersEst.cpp
    src/LidarCornersEst.cpp
    src/Optimization.cpp
    src/Visualization.cpp
)

target_link_libraries(ilcc_lib2 camera_model ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${Sophus_LIBRARIES})


add_executable (get_image_corners_bag test/get_image_corners_bag.cpp )
target_link_libraries (get_image_corners_bag ilcc_lib2  ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable (get_lidar_corners test/get_lidar_corners.cpp )
target_link_libraries (get_lidar_corners ilcc_lib2  ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable (test_calib_lidar_cam test/test_calib_lidar_cam.cpp )
target_link_libraries (test_calib_lidar_cam ilcc_lib2  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable (calib_lidar_cam test/calib_lidar_cam.cpp )
target_link_libraries (calib_lidar_cam ilcc_lib2  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable (pcd2image test/pcd2image.cpp )
target_link_libraries (pcd2image ilcc_lib2  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable (lidar_chessboard_online test/lidar_chessboard_online.cpp )
target_link_libraries (lidar_chessboard_online ilcc_lib2  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable (rgblidar test/rgblidar.cpp )
target_link_libraries (rgblidar ilcc_lib2  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} )

