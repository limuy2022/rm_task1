cmake_minimum_required(VERSION 3.16)
project(team43_lio_backend LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
set(SMALL_LIO_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/vendor/small_point_lio")
# 只替换 ROS 外壳，直接编译上游原始核心；不改第三方文件。
configure_file(${SMALL_LIO_SOURCE}/include/param_deliver.h.in param_deliver.h @ONLY)
add_executable(small_point_lio_node backend.cpp
  ${SMALL_LIO_SOURCE}/src/small_point_lio/small_point_lio.cpp
  ${SMALL_LIO_SOURCE}/src/small_point_lio/parameters.cpp
  ${SMALL_LIO_SOURCE}/src/small_point_lio/preprocess.cpp
  ${SMALL_LIO_SOURCE}/src/small_point_lio/estimator.cpp
  ${SMALL_LIO_SOURCE}/src/util/voxelgrid_sampling.cpp)
target_include_directories(small_point_lio_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SMALL_LIO_SOURCE}/include ${SMALL_LIO_SOURCE}/src
  ${SMALL_LIO_SOURCE}/3rdparty ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(small_point_lio_node Eigen3::Eigen OpenMP::OpenMP_CXX)
ament_target_dependencies(small_point_lio_node rclcpp sensor_msgs nav_msgs livox_ros_driver2)
target_compile_definitions(small_point_lio_node PRIVATE _GLIBCXX_ASSERTIONS)
install(TARGETS small_point_lio_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES upstream.json DESTINATION share/${PROJECT_NAME})
install(FILES "${SMALL_LIO_SOURCE}/LICENSE.txt" DESTINATION share/${PROJECT_NAME})
ament_package()
