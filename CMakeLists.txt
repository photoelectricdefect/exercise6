cmake_minimum_required(VERSION 2.8.3)
project(task3)

## Find system dependencies
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV)

## Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  genmsg
  nodelet
  nodelet_topic_tools
  pcl_conversions
  pcl_msgs
  pluginlib
  rosbag
  rosconsole
  roscpp
  roslib
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
)

## Add include directories
include_directories(
	${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${Boost_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

link_libraries(${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## Declare the catkin package
catkin_package(
  LIBRARIES
    pcl_ros_filters
    pcl_ros_io
    pcl_ros_tf
  CATKIN_DEPENDS
    dynamic_reconfigure
    pcl_conversions
    pcl_msgs
    roscpp
    sensor_msgs
    std_msgs
  DEPENDS
    Boost
    Eigen
    PCL
)


add_executable (ring_cylinder_segmentation_task3 src/ring_cylinder_segmentation.cpp)

#rosbuild_add_executable (cloud_viewer src/cloud_viewer.cpp)
#rosbuild_add_executable (pose_estimation_cluster src/pose_estimation_cluster.cpp)
#add_executable (don_segmentation src/don_segmentation.cpp)

add_executable(save_waypoints_task3 src/save_waypoints.cpp)
target_link_libraries(save_waypoints_task3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(main_task3 src/main.cpp)
target_link_libraries(main_task3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

catkin_install_python(PROGRAMS scripts/parking.py scripts/face_localizer_custom DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})