FROM osrf/ros:humble-desktop-full

RUN apt-get update && \
    apt-get install -q -y --no-install-recommends build-essential autoconf libtool pkg-config cmake && \
    rm -rf /var/lib/apt/lists/*

ENV GRPC_RELEASE_TAG=v1.55.0
ENV GRPC_INSTALL_=/usr/local

RUN wget --directory-prefix=/tmp https://github.com/grpc/grpc/archive/refs/tags/$GRPC_RELEASE_TAG.tar.gz
RUN tar -xzf /tmp/$GRPC_RELEASE_TAG.tar.gz -C /tmp/
RUN mkdir -p /tmp/grpc/cmake/build && cd /tmp/grpc/cmake/build \
    && cmake -DgRPC_INSTALL=ON -DgRPC_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=$GRPC_INSTALL_DIR ../.. \
    && make && make install 

RUN wget --directory-prefix=/tmp https://github.com/googleapis/googleapis/archive/refs/heads/master.tar.gz \
    && tar -xzf /tmp/master.tar.gz -C /tmp/ \
    && find /tmp/googleapis-master/google -type f ! -name "*.proto" -delete \
    && find /tmp/googleapis-master/google -type d -empty -delete \
    && mv -n /tmp/googleapis-master/google $GRPC_INSTALL_DIR/include/ 