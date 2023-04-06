cmake_minimum_required(VERSION 3.5)

project(demo_nodes_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)
# Tutorials of Publish/Subscribe with Topics

add_executable(allocator_tutorial src/allocator_tutorial.cpp)
ament_target_dependencies(allocator_tutorial
  "example_interfaces"
  "rclcpp"
  "rcutils"
  "std_msgs")
install(TARGETS allocator_tutorial
DESTINATION lib/${PROJECT_NAME})


add_library(topics_library SHARED
  src/talker.cpp
  src/listener.cpp)

target_compile_definitions(topics_library PRIVATE "DEMO_NODES_CPP_BUILDING_DLL")
ament_target_dependencies(topics_library
  "example_interfaces"
  "rclcpp"
  "rclcpp_components"
  "rcutils"
  "std_msgs"
  )

rclcpp_components_register_node(topics_library
  PLUGIN "demo_nodes_cpp::Talker"
  EXECUTABLE talker)
rclcpp_components_register_node(topics_library
  PLUGIN "demo_nodes_cpp::Listener"
  EXECUTABLE listener)


install(TARGETS
  topics_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
