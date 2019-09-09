cmake_minimum_required(VERSION 2.8.3)
project(porthos_description)

add_definitions(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
  DEPENDS
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp sensor_msgs gazebo_ros
  LIBRARIES
)
add_executable(wheel_publisher src/wheel_publisher.cpp)
target_link_libraries(wheel_publisher ${catkin_LIBRARIES})
# add_dependencies(wheel_publisher ${Boost_LIBRARIES} ${catkin_EXPORTED_TARGETS})
