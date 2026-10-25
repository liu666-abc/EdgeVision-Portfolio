cmake_minimum_required(VERSION 3.8)
project(evi_cpp_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv4)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  DEPENDENCIES std_msgs
)

add_executable(detector_node src/detector_node.cpp)
target_include_directories(detector_node PRIVATE
  ${OPENCV_INCLUDE_DIRS}
  /usr/include/dnn
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_directories(detector_node PRIVATE ${OPENCV_LIBRARY_DIRS})
target_link_libraries(detector_node
  ${OPENCV_LIBRARIES}
  dnn
  pthread
  rt
  dl
  -Wl,-unresolved-symbols=ignore-in-shared-libs
)
ament_target_dependencies(detector_node rclcpp std_msgs sensor_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(detector_node "${cpp_typesupport_target}")

install(TARGETS detector_node DESTINATION lib/${PROJECT_NAME})

add_executable(tracker_selftest test/tracker_selftest.cpp)
target_include_directories(tracker_selftest PRIVATE
  ${OPENCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_directories(tracker_selftest PRIVATE ${OPENCV_LIBRARY_DIRS})
target_link_libraries(tracker_selftest ${OPENCV_LIBRARIES})
install(TARGETS tracker_selftest DESTINATION lib/${PROJECT_NAME})

add_executable(latency_probe src/latency_probe.cpp)
ament_target_dependencies(latency_probe rclcpp std_msgs)
target_link_libraries(latency_probe "${cpp_typesupport_target}")
install(TARGETS latency_probe DESTINATION lib/${PROJECT_NAME})

ament_package()
