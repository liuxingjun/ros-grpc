FROM liuxingjun/ros:grpc
RUN sed -i s@/archive.ubuntu.com/@/mirrors.tencent.com/@g /etc/apt/sources.list \
    && sed -i s@/security.ubuntu.com/@/mirrors.tencent.com/@g /etc/apt/sources.list \
    && export DEBIAN_FRONTEND=noninteractive && apt-get update 

RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - \
    echo "deb https://mirrors.tuna.tsinghua.edu.cn/llvm-apt/jammy/ llvm-toolchain-jammy main" /etc/apt/sources.list.d/llvm-apt.list

# Update all packages
RUN apt-get -y install wget git python3-pip htop iproute2 ccache gdb clangd \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Rosdep update
# RUN rosdep update && rosdep install --from-paths src --ignore-src -y

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

