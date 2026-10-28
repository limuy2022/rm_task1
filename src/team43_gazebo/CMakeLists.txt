cmake_minimum_required(VERSION 3.16)
project(team43_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 普通工作区只安装资源和辅助模块；运动插件由可选 gazebo-sim 镜像单独编译。
ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY worlds meshes DESTINATION share/${PROJECT_NAME})
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv")
ament_package()
