## build
### 编译 protoc
```
protoc -I ./protos/ --grpc_out include --cpp_out include  --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` protos/*.proto
```
### 编译
```
colcon --log-level debug build --merge-install --packages-up-to ros_grpc --mixin ccache --parallel-workers 6 --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## 启动

```
source install/setup.bash
ros2 run demo_nodes_cpp talker  # 启动ros 消息生产者
ros2 run ros_grpc listener  # 启动ros消息监听者，并开启grpc服务端


```
## debug
```
gdb ./install/lib/ros_grpc/listener core
```