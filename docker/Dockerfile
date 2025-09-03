FROM ubuntu:22.04
LABEL Maintainer="Genki Miyauchi"

ARG DEBIAN_FRONTEND=noninteractive

# Install common dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    dpkg \
    git \
    pkg-config \
    && apt-get clean \
    && rm -r /var/lib/apt/lists/*

# Install ARGoS
RUN apt-get update && apt-get install -y \
    wget \
    cmake \
    freeglut3-dev \
    qt6-base-dev \
    libxi-dev \
    libxmu-dev \
    libfreeimage-dev \
    libfreeimageplus-dev \
    liblua5.3-dev \
    lua5.3 && \
    apt-get clean && \
    rm -r /var/lib/apt/lists/*

# Install Python3
RUN apt-get update && apt-get install -y python3 python3-pip
RUN pip3 install --upgrade pip && \
    pip3 install --no-cache-dir \
    Flask \
    websocket-client

# RUN wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=1gnWfCqNmbCQbxNDenp5-ek7P5bvTNgyD' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1gnWfCqNmbCQbxNDenp5-ek7P5bvTNgyD" -O argos3_simulator.deb && rm -rf /tmp/cookies.txt

RUN pip3 install --no-cache-dir gdown && \
    gdown https://drive.google.com/uc?id=1gnWfCqNmbCQbxNDenp5-ek7P5bvTNgyD -O argos3_simulator.deb && \
    pip3 uninstall -y gdown

RUN apt-get update && apt-get install -y sudo
RUN adduser --disabled-password --gecos '' docker
RUN adduser docker sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER docker
RUN sudo dpkg -i argos3_simulator.deb && sudo rm -rf argos3_simulator.deb
RUN sudo chmod -R ugo+rw /home/docker
USER root

# Install editting tool
RUN apt-get install -y nano

# # Install Protobuf
# RUN apt-get update && apt-get install -y \
#     autoconf \
#     automake \
#     libtool \
#     curl \
#     unzip && \
#     apt-get clean && \
#     rm -r /var/lib/apt/lists/*

# RUN wget https://github.com/protocolbuffers/protobuf/releases/download/v21.12/protobuf-cpp-3.21.12.tar.gz && \
#     tar xzf protobuf-cpp-3.21.12.tar.gz && \
#     cd protobuf-3.21.12/ && \
#     ./configure && \
#     make -j$(nproc) && \
#     make install && \
#     ldconfig && \
#     cd / && \
#     rm protobuf-cpp-3.21.12.tar.gz && \
#     rm -rf protobuf-3.21.12

# Install yaml
RUN apt-get update && apt-get install -y libyaml-cpp-dev

# Install plugins for ARGoS
RUN apt-get install -y nlohmann-json3-dev

# ADD "https://gitlab.com/api/v4/projects/34344095/repository/branches/main" latest_info
# RUN git clone https://gitlab.com/genki_miyauchi/argos-sct-plugins.git /home/docker/argos-sct-plugins && \
#     cd /home/docker/argos-sct-plugins && \
#     mkdir build && \
#     cd build && \
#     cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr ../src &&\
#     make && \
#     make install && \
#     cd ../.. && \
#     rm -rf argos-sct-plugins

# Install Webviz
RUN apt-get update && apt-get install -y \
    zlib1g-dev \
    libssl-dev

ADD "https://gitlab.com/api/v4/projects/31175600/repository/branches/swarm-perception" latest_info
RUN git clone -b swarm-perception https://gitlab.com/genki_miyauchi/argos3-webviz.git /home/docker/argos3-webviz &&\
    cd /home/docker/argos3-webviz && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr ../src &&\
    make && \
    make install && \
    cd ../.. && \
    rm -rf argos3-webviz

# Install swarm-perception
ADD "https://api.github.com/repos/genkimiyauchi/swarm-perception/branches/main" latest_info
RUN git clone -b main https://github.com/genkimiyauchi/swarm-perception.git /home/docker/swarm-perception &&\
    cd /home/docker/swarm-perception && \
    # mkdir results && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr .. &&\
    make && \
    cd .. && \
    rm -rf \
    .git \
    .vscode \
    docker \
    src/controllers \
    src/loop_functions \
    src/protos \
    src/utility \
    src/scripts

RUN apt-get autoremove -y &&\
    apt-get clean &&\
    rm -rf /usr/local/src/*

# WORKDIR /home/docker/argos-sct
# ENTRYPOINT [ "python3", "src/scripts/app/app.py -m indirect -o 1" ]
