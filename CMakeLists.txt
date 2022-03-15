cmake_minimum_required(VERSION 3.0.2)
project(drdo-interiit22)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES interiit22-drdo
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(init_drone src/path_planning/init_drone.cpp)

target_link_libraries(init_drone
  ${catkin_LIBRARIES}
)
