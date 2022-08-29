cmake_minimum_required(VERSION 3.0.2)
project(rrtstar_msgs)


find_package(catkin REQUIRED COMPONENTS
std_msgs
geometry_msgs
message_generation
actionlib_msgs)

add_service_files(
  DIRECTORY srv
  FILES
  velocity.srv
  build_tree.srv
  search.srv
  pci_search.srv
)


generate_messages(
  DEPENDENCIES
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs
)

