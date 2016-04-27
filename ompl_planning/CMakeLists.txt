cmake_minimum_required(VERSION 2.8.3)
project(ompl_planning)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib
      controller_interface
      hardware_interface
      realtime_tools
      control_msgs
      trajectory_msgs
      cmake_modules
      sensor_msgs
      nav_msgs
      tf
      geometry_msgs
      controller_manager
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OMPL)

catkin_package(
  CATKIN_DEPENDS actionlib cmake_modules control_msgs controller_interface hardware_interface realtime_tools
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

add_executable(porthos_base_planner_class src/porthos_base_planner_class.cpp)

target_link_libraries(
  porthos_base_planner_class
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)
add_executable(measureEEF src/measureEEF.cpp)

target_link_libraries(
  measureEEF
  ${catkin_LIBRARIES}
)
