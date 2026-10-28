cmake_minimum_required(VERSION 3.16)
project(team43_gazebo_plugins LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
add_library(rm_task1_force_controller SHARED force_controller.cpp)
target_link_libraries(rm_task1_force_controller PRIVATE
  ignition-gazebo6::ignition-gazebo6 ignition-plugin1::ignition-plugin1)
install(TARGETS rm_task1_force_controller LIBRARY DESTINATION lib)
