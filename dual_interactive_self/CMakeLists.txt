cmake_minimum_required(VERSION 2.8.3)
project(dual_interactive_self)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib_msgs actionlib moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules)


find_package(Boost REQUIRED system filesystem date_time thread)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES our_robot ????????
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



add_executable(dual_interactive_self src/dual_interactive_self.cpp)
target_link_libraries(dual_interactive_self ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS dual_interactive_self DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
