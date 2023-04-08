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
ros2 launch ros_grpc talker_listener.launch.xml
```
## 使用 grpc 客户端测试
### 编译
```
cd /var/local/grpc/examples/cpp/helloworld
mkdir -p cmake/build && cd cmake/build
cmake -DCMAKE_PREFIX_PATH=~/.grpc ../..
make
```
### 运行
```
./greeter_client
```
## 
## debug
```
gdb ./install/lib/ros_grpc/listener core
```