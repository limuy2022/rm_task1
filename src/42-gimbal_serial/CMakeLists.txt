cmake_minimum_required(VERSION 3.8)
project(gimbal_serial_42)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

# 串口封装：只依赖 POSIX，不依赖 ROS
add_library(${PROJECT_NAME}_serial STATIC src/serial_port.cpp)
target_include_directories(${PROJECT_NAME}_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(${PROJECT_NAME}_serial PROPERTIES POSITION_INDEPENDENT_CODE ON)

# 桥节点
add_library(${PROJECT_NAME}_core STATIC src/bridge_node.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(${PROJECT_NAME}_core PUBLIC ${PROJECT_NAME}_serial)
ament_target_dependencies(${PROJECT_NAME}_core PUBLIC
  rclcpp sensor_msgs std_msgs geometry_msgs builtin_interfaces tf2_ros)

add_executable(robo_bridge_node src/robo_bridge_node_main.cpp)
target_link_libraries(robo_bridge_node ${PROJECT_NAME}_core)

install(TARGETS robo_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# 相机由现场已有驱动发布；本包只构建串口桥，不提供缺失的相机SDK源码目标。

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 黄金向量在仓库根的 protocol/ 下（git submodule gimbal-edu-protocol），包内不放副本，避免两份数据漂移。
  # 按顺序查找：环境变量 GOLDEN_FRAMES_DIR；本包所在仓库根的 protocol/（包目录的 ../../protocol）。
  set(_golden_dirs $ENV{GOLDEN_FRAMES_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../protocol)
  set(GOLDEN_FRAMES_PATH "")
  foreach(_d ${_golden_dirs})
    if(NOT GOLDEN_FRAMES_PATH AND EXISTS "${_d}/golden_frames.txt")
      get_filename_component(GOLDEN_FRAMES_PATH "${_d}/golden_frames.txt" ABSOLUTE)
    endif()
  endforeach()
  if(NOT GOLDEN_FRAMES_PATH)
    # 不让 build 失败：依赖黄金向量的用例在运行时 GTEST_SKIP，并给出醒目提示。
    message(WARNING "\n"
      "!!! 找不到 golden_frames.txt：黄金向量测试将被【跳过】，协议解码没有被验证 !!!\n"
      "    请先在仓库里执行 git submodule update --init（克隆时加 --recursive），\n"
      "    或设置环境变量 GOLDEN_FRAMES_DIR 指向 protocol/ 目录后重新 cmake。")
  else()
    message(STATUS "golden_frames.txt: ${GOLDEN_FRAMES_PATH}")
  endif()

  ament_add_gtest(${PROJECT_NAME}_protocol_test test/protocol_test.cpp)
  target_link_libraries(${PROJECT_NAME}_protocol_test ${PROJECT_NAME}_serial)
  target_include_directories(${PROJECT_NAME}_protocol_test PRIVATE include)
  target_compile_definitions(${PROJECT_NAME}_protocol_test PRIVATE
    GOLDEN_FRAMES_PATH="${GOLDEN_FRAMES_PATH}")
  ament_target_dependencies(${PROJECT_NAME}_protocol_test
    sensor_msgs geometry_msgs builtin_interfaces)

  ament_add_gtest(${PROJECT_NAME}_serial_test test/serial_port_test.cpp)
  target_link_libraries(${PROJECT_NAME}_serial_test ${PROJECT_NAME}_serial)

  # 节点集成测试：pty 充当串口，节点与测试节点同进程
  ament_add_gtest(${PROJECT_NAME}_node_test test/node_test.cpp
    SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 120)
  target_link_libraries(${PROJECT_NAME}_node_test ${PROJECT_NAME}_core gtest)
  ament_target_dependencies(${PROJECT_NAME}_node_test rclcpp sensor_msgs geometry_msgs tf2_msgs)


endif()

ament_package()
