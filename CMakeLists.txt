cmake_minimum_required(VERSION 3.8)

project(ros_grpc C CXX)

include(cmake/common.cmake) 
# Proto file

# Generated sources
set(hw_proto_srcs "include/helloworld.pb.cc")
set(hw_proto_hdrs "include/helloworld.pb.h")
set(hw_grpc_srcs "include/helloworld.grpc.pb.cc")
set(hw_grpc_hdrs "include/helloworld.grpc.pb.h")

# Include generated *.pb.h files
include_directories(include)

# hw_grpc_proto
add_library(hw_grpc_proto
  ${hw_grpc_srcs}
  ${hw_grpc_hdrs}
  ${hw_proto_srcs}
  ${hw_proto_hdrs})
target_link_libraries(hw_grpc_proto
  ${_REFLECTION}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF})


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(listener
	src/listener.cpp
)
target_link_libraries(listener
	hw_grpc_proto
	${_REFLECTION}
	${_GRPC_GRPCPP}
	${_PROTOBUF_LIBPROTOBUF})


ament_target_dependencies(listener
	"rclcpp"
	"std_msgs"
)

install(TARGETS
	listener
	DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
)

ament_package()