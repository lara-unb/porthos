cmake_minimum_required(VERSION 2.8.3)
project(porthos_planning)

find_package(catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pr2_moveit_tutorials
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)
find_package(Eigen REQUIRED)

###########
## Build ##
###########


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(porthos_moves src/porthos_moves.cpp)
target_link_libraries(porthos_moves ${catkin_LIBRARIES})
add_dependencies(porthos_moves ${Boost_LIBRARIES} ${catkin_EXPORTED_TARGETS})

add_executable(porthos_benchmark src/porthos_benchmark.cpp)
target_link_libraries(porthos_benchmark ${catkin_LIBRARIES})
add_dependencies(porthos_benchmark ${Boost_LIBRARIES} ${catkin_EXPORTED_TARGETS})
