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
  rclcpp sensor_msgs geometry_msgs builtin_interfaces tf2_ros)

add_executable(robo_bridge_node src/robo_bridge_node_main.cpp)
target_link_libraries(robo_bridge_node ${PROJECT_NAME}_core)

install(TARGETS robo_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# 可选：海康相机驱动，引用系统安装的 MVS SDK（deb 默认装在 /opt/MVS）。
# 本副本（42-gimbal_serial）不携带 hik_camera_node 的源码，赛题一的相机由主办方启动，
# 因此默认 OFF；AUTO/ON 模式下发现源码缺失也会跳过/报错，绝不在装有 MVS SDK 的机器上
# 因为找不到源文件而编译失败。
#   WITH_HIK_CAMERA=OFF（默认）：不编译
#   WITH_HIK_CAMERA=AUTO         ：SDK、ROS 依赖和源码都齐全才编译，缺任何一项都静默跳过（只打 STATUS）
#   WITH_HIK_CAMERA=ON           ：缺任何一项都 FATAL_ERROR
set(WITH_HIK_CAMERA "OFF" CACHE STRING "Build hik_camera_node against the system MVS SDK: AUTO, ON or OFF")
set_property(CACHE WITH_HIK_CAMERA PROPERTY STRINGS AUTO ON OFF)
string(TOUPPER "${WITH_HIK_CAMERA}" _hik_mode)
if(NOT _hik_mode MATCHES "^(AUTO|ON|OFF)$")
  message(FATAL_ERROR "WITH_HIK_CAMERA must be AUTO, ON or OFF (got '${WITH_HIK_CAMERA}')")
endif()

# MVS_ROOT 默认取环境变量 MVCAM_SDK_PATH（MVS 的安装脚本会写进 /etc/profile），否则 /opt/MVS
set(_mvs_root_default "/opt/MVS")
if(NOT "$ENV{MVCAM_SDK_PATH}" STREQUAL "" AND IS_DIRECTORY "$ENV{MVCAM_SDK_PATH}")
  set(_mvs_root_default "$ENV{MVCAM_SDK_PATH}")
endif()
set(MVS_ROOT "${_mvs_root_default}" CACHE PATH "Root directory of the system MVS SDK (contains include/ and lib/)")

set(HIK_CAMERA_ENABLED FALSE)
if(_hik_mode STREQUAL "OFF")
  message(STATUS "hik_camera: skipped (WITH_HIK_CAMERA=OFF)")
else()
  set(_hik_reason "")

  # 源码缺失：本副本未携带 hik_camera_node / hik_sdk_test 的源文件，
  # 即使机器上装了 MVS SDK（比赛机就装了）也绝不能进入编译分支。
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/hik_camera_node.cpp")
    set(_hik_reason "src/hik_camera_node.cpp not present in this copy")
  endif()

  # 架构子目录
  set(MVS_LIB_DIR "")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$")
    set(MVS_LIB_DIR "${MVS_ROOT}/lib/64")
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(MVS_LIB_DIR "${MVS_ROOT}/lib/aarch64")
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^armv7")
    set(MVS_LIB_DIR "${MVS_ROOT}/lib/armhf")
  else()
    set(_hik_reason "unsupported architecture '${CMAKE_SYSTEM_PROCESSOR}'")
  endif()

  # MVS_ROOT 或架构变了，或者缓存的路径已经不存在（例如卸载了 SDK）：已缓存的 find 结果必须失效，
  # 否则会沿用旧路径。
  set(_mvs_key "${MVS_ROOT}|${MVS_LIB_DIR}")
  set(_mvs_stale FALSE)
  if(NOT "${_MVS_SEARCH_KEY}" STREQUAL "${_mvs_key}")
    set(_mvs_stale TRUE)
  endif()
  foreach(_v MVS_INCLUDE_DIR MVS_CAMERA_CONTROL_LIBRARY)
    if(${_v} AND NOT EXISTS "${${_v}}")
      set(_mvs_stale TRUE)
    endif()
  endforeach()
  if(_mvs_stale)
    unset(MVS_INCLUDE_DIR CACHE)
    unset(MVS_CAMERA_CONTROL_LIBRARY CACHE)
  endif()
  set(_MVS_SEARCH_KEY "${_mvs_key}" CACHE INTERNAL "MVS_ROOT and lib dir the cached find results belong to")

  if(NOT _hik_reason)
    find_path(MVS_INCLUDE_DIR MvCameraControl.h PATHS ${MVS_ROOT}/include NO_DEFAULT_PATH)
    find_library(MVS_CAMERA_CONTROL_LIBRARY MvCameraControl PATHS ${MVS_LIB_DIR} NO_DEFAULT_PATH)
    if(NOT MVS_INCLUDE_DIR)
      set(_hik_reason "MvCameraControl.h not found under ${MVS_ROOT}/include")
    elseif(NOT MVS_CAMERA_CONTROL_LIBRARY)
      set(_hik_reason "libMvCameraControl.so not found under ${MVS_LIB_DIR}")
    endif()
  endif()

  if(NOT _hik_reason)
    if(_hik_mode STREQUAL "ON")
      find_package(rclcpp_components REQUIRED)
      find_package(image_transport REQUIRED)
      find_package(camera_info_manager REQUIRED)
    else()
      find_package(rclcpp_components QUIET)
      find_package(image_transport QUIET)
      find_package(camera_info_manager QUIET)
      foreach(_p rclcpp_components image_transport camera_info_manager)
        if(NOT ${_p}_FOUND AND NOT _hik_reason)
          set(_hik_reason "ROS package ${_p} not found")
        endif()
      endforeach()
    endif()
  endif()

  if(_hik_reason)
    if(_hik_mode STREQUAL "ON")
      message(FATAL_ERROR "WITH_HIK_CAMERA=ON but the hik camera driver cannot be built: ${_hik_reason}.\n"
        "Install the MVS deb (default /opt/MVS), or pass -DMVS_ROOT=<MVS install dir>; "
        "use -DWITH_HIK_CAMERA=OFF to skip the driver.")
    endif()
    message(STATUS "hik_camera: skipped, ${_hik_reason}. "
      "To enable: install the MVS deb (default /opt/MVS, or -DMVS_ROOT=<dir>) and the ROS packages "
      "rclcpp_components, image_transport, camera_info_manager.")
  else()
    set(HIK_CAMERA_ENABLED TRUE)
    get_filename_component(_mvs_lib_real "${MVS_CAMERA_CONTROL_LIBRARY}" REALPATH)
    set(_mvs_lib_version "unknown")
    if(_mvs_lib_real MATCHES "\\.so\\.([0-9]+(\\.[0-9]+)*)$")
      set(_mvs_lib_version "${CMAKE_MATCH_1}")
    endif()
    message(STATUS "hik_camera: enabled, MVS SDK ${MVS_CAMERA_CONTROL_LIBRARY} -> ${_mvs_lib_real} "
      "(file version ${_mvs_lib_version}), headers ${MVS_INCLUDE_DIR}")

    add_library(${PROJECT_NAME}_hik_camera SHARED src/hik_camera_node.cpp)
    ament_target_dependencies(${PROJECT_NAME}_hik_camera
      rclcpp rclcpp_components image_transport camera_info_manager sensor_msgs)
    # SYSTEM：屏蔽厂商头文件的警告
    target_include_directories(${PROJECT_NAME}_hik_camera SYSTEM PRIVATE ${MVS_INCLUDE_DIR})
    # 只链接 MvCameraControl，其他库（MvUsb3vTL 等）由 SDK 自己加载
    # rpath-link 用链接项传入（target_link_options 要 CMake 3.13，本包声明的是 3.8）
    target_link_libraries(${PROJECT_NAME}_hik_camera ${MVS_CAMERA_CONTROL_LIBRARY} ${CMAKE_DL_LIBS}
      "-Wl,-rpath-link,${MVS_LIB_DIR}")
    # RUNPATH 用稳定路径（不带版本号）
    set_target_properties(${PROJECT_NAME}_hik_camera PROPERTIES
      BUILD_RPATH "${MVS_LIB_DIR}" INSTALL_RPATH "${MVS_LIB_DIR}")
    rclcpp_components_register_node(${PROJECT_NAME}_hik_camera
      PLUGIN "hik_camera::HikCameraNode" EXECUTABLE hik_camera_node)
    install(TARGETS ${PROJECT_NAME}_hik_camera
      ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  endif()
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 黄金向量在仓库根的 protocol/ 下（git submodule gimbal-edu-protocol），包内不放副本，避免两份数据漂移。
  # 按顺序查找：环境变量 GOLDEN_FRAMES_DIR；本包所在仓库根的 protocol/（包目录的 ../protocol）。
  set(_golden_dirs $ENV{GOLDEN_FRAMES_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../protocol)
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

  # 海康 SDK 测试：只在编译了海康驱动时才加
  if(HIK_CAMERA_ENABLED)
    ament_add_gtest(${PROJECT_NAME}_hik_sdk_test test/hik_sdk_test.cpp)
    target_include_directories(${PROJECT_NAME}_hik_sdk_test SYSTEM PRIVATE ${MVS_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME}_hik_sdk_test ${MVS_CAMERA_CONTROL_LIBRARY} ${CMAKE_DL_LIBS}
      "-Wl,-rpath-link,${MVS_LIB_DIR}")
    set_target_properties(${PROJECT_NAME}_hik_sdk_test PROPERTIES
      BUILD_RPATH "${MVS_LIB_DIR}" INSTALL_RPATH "${MVS_LIB_DIR}")
    target_compile_definitions(${PROJECT_NAME}_hik_sdk_test PRIVATE
      MVS_LIB_DIR="${MVS_LIB_DIR}"
      HIK_COMPONENT_LIB="$<TARGET_FILE:${PROJECT_NAME}_hik_camera>")
    # dlopen 组件库时它必须已经编译好
    add_dependencies(${PROJECT_NAME}_hik_sdk_test ${PROJECT_NAME}_hik_camera)
  endif()
endif()

ament_package()
