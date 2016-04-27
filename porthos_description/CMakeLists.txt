cmake_minimum_required(VERSION 2.8.3)
project(porthos_description)

find_package(catkin REQUIRED COMPONENTS gazebo_ros)

catkin_package(
  DEPENDS
  INCLUDE_DIRS
  CATKIN_DEPENDS 
  LIBRARIES
)
add_executable(wheel_publisher src/wheel_publisher.cpp)
target_link_libraries(wheel_publisher ${catkin_LIBRARIES})
add_dependencies(wheel_publisher ${Boost_LIBRARIES} ${catkin_EXPORTED_TARGETS})
