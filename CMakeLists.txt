cmake_minimum_required(VERSION 3.0.2)
project(pr2_gripper_perception_module)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  overworld
  pluginlib
  pr2_msgs
  roscpp
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/Pr2GripperPerceptionModule.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC
                      ${catkin_LIBRARIES}
                      ${overworld_LIBRARIES})