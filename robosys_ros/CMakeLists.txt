cmake_minimum_required(VERSION 2.8.3)
project(robosys_ros)

find_package(
  catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nodelet
  message_generation
)

add_message_files(
  FILES
  BallDetection.msg
  BallDetectionArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Red.cfg
  cfg/Blue.cfg
  cfg/Yellow.cfg
  cfg/Ball.cfg
)

catkin_package(
  CATKIN_DEPENDS
)

add_compile_options(
  "-std=c++14"
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)


#add_library(ball_detector
#  src/ball_detector.cpp
#)
#target_link_libraries(ball_detector ${catkin_LIBRARIES})

add_executable(ball_detector_node
  src/ball_detector.cpp
  src/ball_detector_node.cpp
  src/color_ball.cpp
)
target_link_libraries(ball_detector_node ${catkin_LIBRARIES})

add_library(robosys_ros
  src/ball_detector.cpp
  src/ball_detector_nodelet.cpp
  src/color_ball.cpp
)
target_link_libraries(robosys_ros ${catkin_LIBRARIES})


add_executable(red_threshold
  src/reconfigure/red_threshold.cpp)
add_dependencies(red_threshold
  ${PROJECT_NAME}_gencfg)
target_link_libraries(red_threshold ${catkin_LIBRARIES})

add_executable(blue_threshold
  src/reconfigure/blue_threshold.cpp)
add_dependencies(blue_threshold
  ${PROJECT_NAME}_gencfg)
target_link_libraries(blue_threshold ${catkin_LIBRARIES})

add_executable(yellow_threshold
  src/reconfigure/yellow_threshold.cpp)
add_dependencies(yellow_threshold
  ${PROJECT_NAME}_gencfg)
target_link_libraries(yellow_threshold ${catkin_LIBRARIES})

add_executable(ball_threshold
  src/reconfigure/ball_threshold.cpp)
add_dependencies(ball_threshold
  ${PROJECT_NAME}_gencfg)
target_link_libraries(ball_threshold ${catkin_LIBRARIES})
