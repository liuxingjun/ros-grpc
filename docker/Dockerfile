FROM osrf/ros:humble-desktop-full

RUN apt-get update && \
    apt-get install -q -y --no-install-recommends build-essential autoconf libtool pkg-config cmake && \
    rm -rf /var/lib/apt/lists/*

ENV GRPC_RELEASE_TAG=v1.53.0
ENV GRPC_INSTALL_DIR=~/.grpc

RUN git clone -b ${GRPC_RELEASE_TAG} --recurse-submodules --depth 1 --shallow-submodules https://github.com/grpc/grpc /var/local/grpc 
RUN cd /var/local/grpc && mkdir -p cmake/build && cd cmake/build \
    && cmake -DgRPC_INSTALL=ON -DgRPC_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=$GRPC_INSTALL_DIR ../.. \
    && make && make install 
RUN echo "PATH=$GRPC_INSTALL_DIR/bin:$PATH" >> ~/.bashrc