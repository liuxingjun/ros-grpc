// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>

#include <iostream>
#include <memory>
#include <string>

#include "helloworld.grpc.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using helloworld::Greeter;
using helloworld::HelloReply;
using helloworld::HelloRequest;

// Logic and data behind the server's behavior.

// Logic and data behind the server's behavior.
/// LifecycleListener class as a simple listener node
/**
 * We subscribe to two topics
 * - lifecycle_chatter: The data topic from the talker
 * - lc_talker__transition_event: The topic publishing
 *   notifications about state changes of the node
 *   lc_talker
 */
class GreeterServiceImpl final : public Greeter::Service {
 public:
  std::string msg;
  Status SayHello(ServerContext* context, const HelloRequest* request,
                  HelloReply* reply) override {
    std::string prefix("Hello ");
    reply->set_message(prefix + request->name());
    return Status::OK;
  }
};

class LifecycleListener : public rclcpp::Node {
 public:
  explicit LifecycleListener(const std::string& node_name) : Node(node_name) {
    // Data topic from the lc_talker node
    sub_data_ = this->create_subscription<std_msgs::msg::String>(
        "chatter", 10,
        std::bind(&LifecycleListener::data_callback, this,
                  std::placeholders::_1));
    std::thread t([this]() {
      // 执行需要在新线程中运行的操作
      std::string server_address("0.0.0.0:50051");
      GreeterServiceImpl service;
      grpc::EnableDefaultHealthCheckService(true);
      grpc::reflection::InitProtoReflectionServerBuilderPlugin();
      ServerBuilder builder;
      builder.AddListeningPort(server_address,
                               grpc::InsecureServerCredentials());
      builder.RegisterService(&service);
      std::unique_ptr<Server> server(builder.BuildAndStart());
      std::cout << "Server listening on " << server_address << std::endl;
      server->Wait();
    });
    t.detach();  // 让新线程在后台运行
  }

  void data_callback(std_msgs::msg::String::ConstSharedPtr msg) {
    RCLCPP_INFO(get_logger(), "data_callback: %s", msg->data.c_str());
  }

 private:
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub_data_;
  std::shared_ptr<GreeterServiceImpl> service_;
};

int mains(int argc, char** argv) {
  std::string server_address("0.0.0.0:50051");
  GreeterServiceImpl service;
  grpc::EnableDefaultHealthCheckService(true);
  grpc::reflection::InitProtoReflectionServerBuilderPlugin();
  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;
  server->Wait();
}
int main(int argc, char** argv) {
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto lc_listener = std::make_shared<LifecycleListener>("lc_listener");
  rclcpp::spin(lc_listener);

  rclcpp::shutdown();
  std::cout << "Server listening on " << std::endl;
  // RunServer()
  return 0;
}