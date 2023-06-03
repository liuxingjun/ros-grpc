FROM osrf/ros:humble-desktop-full

RUN apt-get update && \
    apt-get install -q -y --no-install-recommends build-essential autoconf libtool pkg-config cmake && \
    rm -rf /var/lib/apt/lists/*

ENV GRPC_RELEASE_TAG=v1.53.0
ENV GRPC_DOWNLOAD_DIR=/tmp/grpc

RUN git clone -b ${GRPC_RELEASE_TAG} --recurse-submodules --depth 1 --shallow-submodules https://github.com/grpc/grpc $GRPC_DOWNLOAD_DIR
RUN cd $GRPC_DOWNLOAD_DIR && mkdir -p cmake/build && cd cmake/build \
    && cmake -DgRPC_INSTALL=ON -DgRPC_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=/usr/local ../.. \
    && make && make install 