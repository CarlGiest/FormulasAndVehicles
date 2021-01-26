cmake_minimum_required(VERSION 2.8.3)
project(bluerov_sim)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  roscpp
  std_msgs
  message_generation
  range_sensor
)

add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(gazebo REQUIRED)

add_message_files(
    FILES
    ActuatorCommands.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS}
                            ${GAZEBO_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIR})

catkin_package(
#  INCLUDE_DIRS include
  DEPENDS 
      roscpp 
      gazebo_ros
  CATKIN_DEPENDS 
    message_runtime
)

add_library(gazebo_pressure_plugin plugins/gazebo_pressure_plugin.cpp)
add_dependencies(gazebo_pressure_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_pressure_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_range_sensor_plugin plugins/gazebo_range_sensor_plugin.cpp)
add_dependencies(gazebo_range_sensor_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_range_sensor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

