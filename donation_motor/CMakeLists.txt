cmake_minimum_required(VERSION 2.8.3)
project(donation_motor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES donation_motor
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(motor_node src/motor_node.cpp)
add_dependencies(motor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_node ${catkin_LIBRARIES} pigpiod_if2)

add_executable(button_node src/button_node.cpp)
add_dependencies(button_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(button_node ${catkin_LIBRARIES} pigpiod_if2)
