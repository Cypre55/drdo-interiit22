cmake_minimum_required(VERSION 3.0.2)
project(drdo_interiit22)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  nav_msgs
  geometry_msgs
  prius_msgs
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  customMessage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs

)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES interiit22-drdo
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(control2 control2.cpp)

target_link_libraries(control2
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(car_control src/UGV_localisation/control2_path_generation.cpp)

target_link_libraries(car_control
  ${catkin_LIBRARIES}
)
