cmake_minimum_required(VERSION 2.8.3)
project(kamui_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
#  sh2interface
  std_msgs
  message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  flipper_velocity.msg
  base_velocity.msg
  robot_state.msg
)

generate_messages(
  DEPENDENCIES
#  geometry_msgs
  std_msgs
#  sh2interface
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kamui_control
#  CATKIN_DEPENDS geometry_msgs roscpp roscpp sh2interface std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(kamui_control_node src/kamui_control_node.cpp)

target_link_libraries(kamui_control_node ${catkin_LIBRARIES})
