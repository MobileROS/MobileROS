FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       build-essential \
       ca-certificates \
       cmake \
       git \
       python3 \
       python3-pip \
       libzmq3-dev \
       libprotobuf-dev \
       protobuf-compiler \
       libsctp-dev \
       libboost-all-dev \
       libfftw3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/mobileros
COPY . /opt/mobileros

RUN cmake -S . -B build -DENABLE_OAI=OFF \
    && cmake --build build \
    && cmake --install build

ENV PATH="/usr/local/bin:${PATH}"
ENTRYPOINT ["/opt/mobileros/docker/entrypoint.sh"]
CMD ["mobileros_example"]
