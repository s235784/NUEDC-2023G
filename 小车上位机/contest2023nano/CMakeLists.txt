cmake_minimum_required(VERSION 3.0.2)
project(contest2023nano)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES contest2023nano
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

find_package(OpenCV 4.6.0 REQUIRED)

find_package(realsense2 REQUIRED
  PATHS /usr/local/lib/cmake/realsense2
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${realsense_INCLUDE_DIR}
)

add_executable(
  contest_car
  src/Car.cpp
)
add_dependencies(contest_car ${catkin_EXPORTED_TARGETS})
target_link_libraries(contest_car ${realsense2_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(
  contest_stop
  src/CarStop.cpp
)
add_dependencies(contest_stop ${catkin_EXPORTED_TARGETS})
target_link_libraries(contest_stop ${realsense2_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/devices.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
