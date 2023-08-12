cmake_minimum_required(VERSION 3.0.2)
project(my_utils)

set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  serial
  message_generation
  mavros_msgs
  cv_bridge
)

# set(OpenCV_DIR "/home/jetson/OpenCV/opencv/build/")
# set(OpenCV_INCLUDE_DIRS "/home/jetson/OpenCV/opencv/build/include")
# find_package(OpenCV 4.8.0)
find_package(OpenCV 4.8.0 REQUIRED
  COMPONENTS
    opencv_core
    opencv_highgui
    opencv_imgproc
    opencv_imgcodecs
)

add_message_files(
  FILES
  my_led.msg
  my_led_unique.msg
  my_location.msg
  my_servo.msg
  my_hook.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs roscpp message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# add_executable(t265_compass src/T265Compass.cpp)
# add_dependencies(t265_compass ${catkin_EXPORTED_TARGETS})
# target_link_libraries(t265_compass ${catkin_LIBRARIES})

# add_executable(code src/code.cpp)
# add_dependencies(code ${catkin_EXPORTED_TARGETS} ${OpenCV_LIBRARIES})
# target_link_libraries(code ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable(yolo src/yolo.cpp)
# add_dependencies(yolo ${catkin_EXPORTED_TARGETS} ${OpenCV_LIBRARIES})
# target_link_libraries(yolo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(utils_test src/test.cpp)
add_dependencies(utils_test ${catkin_EXPORTED_TARGETS} ${OpenCV_LIBRARIES})
target_link_libraries(utils_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable(led src/led.cpp)
# add_dependencies(led ${catkin_EXPORTED_TARGETS})
# target_link_libraries(led ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/devicesContest.py
  scripts/codeCamera.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
