cmake_minimum_required(VERSION 3.0.2)
project(rimapp_msgs)


find_package(catkin REQUIRED COMPONENTS
std_msgs
geometry_msgs
message_generation
actionlib_msgs)

add_service_files(
  DIRECTORY srv
  FILES
  plan_path_single.srv
  pci_plan_path_single.srv
  upi_planner.srv
  upi_ma_planner.srv
)

add_message_files(
  DIRECTORY msg
  FILES
  Bestpath.msg
  UpiPlanner.msg
)


generate_messages(
  DEPENDENCIES
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs
)

