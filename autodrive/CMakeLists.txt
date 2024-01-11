cmake_minimum_required(VERSION 3.0.2)
project(autodrive)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
 
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES autodrive
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


# add_executable(joystick_node src/joystick_node.cpp)
add_executable(command_node src/command_node.cpp)
add_executable(plannif_node src/plannif_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(joystick_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(command_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(plannif_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Specify libraries to link a library or executable target against
# target_link_libraries(joystick_node ${catkin_LIBRARIES})
target_link_libraries(command_node ${catkin_LIBRARIES})
target_link_libraries(plannif_node ${catkin_LIBRARIES})



#############
## Install ##
#############
