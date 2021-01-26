cmake_minimum_required(VERSION 3.0.2)
project(depth_controller)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

# add_message_files(
#     FILES
# )

# generate_messages(
#     DEPENDENCIES
#     std_msgs
#)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES depth_controller
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
