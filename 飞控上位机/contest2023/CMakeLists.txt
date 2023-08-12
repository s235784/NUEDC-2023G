cmake_minimum_required(VERSION 3.0.2)
project(contest2023)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mavros_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  my_utils
  detection_msgs
)

add_message_files(
  FILES
  camera_offset.msg
  camera_yaw_fix.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(OpenCV 4.8.0 REQUIRED)

find_package(realsense2 REQUIRED
  PATHS /usr/local/lib/cmake/realsense2
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES example2
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${realsense_INCLUDE_DIR}
)

add_executable(
  contest_pose
  src/PoseNew.cpp
)
add_dependencies(contest_pose ${catkin_EXPORTED_TARGETS})
target_link_libraries(contest_pose ${realsense2_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(
  contest_camera
  src/Camera.cpp
)
add_dependencies(contest_camera ${catkin_EXPORTED_TARGETS})
target_link_libraries(contest_camera ${realsense2_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS})

add_executable(
  contest_yolotest
  src/YoloTest.cpp
)
add_dependencies(contest_yolotest ${catkin_EXPORTED_TARGETS})
target_link_libraries(contest_yolotest ${realsense2_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})