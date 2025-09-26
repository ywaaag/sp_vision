# syntax=docker/dockerfile:1

# ========= 第一阶段：基础构建环境 =========
# 使用一个基础镜像，例如 Ubuntu 22.04 LTS
FROM ubuntu:22.04 AS builder

ARG DEBIAN_FRONTEND=noninteractive
ARG http_proxy
ARG https_proxy
ENV http_proxy=$http_proxy
ENV https_proxy=$https_proxy

# 确保 apt-get 在非交互模式下运行，并更新源
RUN apt-get update && apt-get install -y ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# 🚀 换源，确保国内下载速度
RUN sed -i 's|http://archive.ubuntu.com|https://mirrors.aliyun.com|g' /etc/apt/sources.list && \
    sed -i 's|http://security.ubuntu.com|https://mirrors.aliyun.com|g' /etc/apt/sources.list

# 安装所有必需的依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential git g++ cmake wget curl unzip \
    can-utils libopencv-dev libfmt-dev libeigen3-dev \
    libspdlog-dev libyaml-cpp-dev libusb-1.0-0-dev \
    libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libsuitesparse-dev \
    software-properties-common && \
    rm -rf /var/lib/apt/lists/*

# 解决 CMake 找不到头文件库的问题
RUN mkdir -p /usr/local/include/nlohmann && \
    curl -L https://github.com/nlohmann/json/releases/latest/download/json.hpp -o /usr/local/include/nlohmann/json.hpp

WORKDIR /tmp

# ========= 第二阶段：Ceres Solver 安装 =========
FROM builder AS ceres_stage

ARG http_proxy
ARG https_proxy
ENV http_proxy=$http_proxy
ENV https_proxy=$https_proxy

# 编译和安装 Ceres Solver
RUN git clone -b 2.1.0 https://github.com/ceres-solver/ceres-solver.git && \
    cd ceres-solver && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/ceres-solver

# 更新动态链接库缓存
RUN ldconfig && \
    echo "/usr/local/lib" >> /etc/ld.so.conf.d/ceres.conf && \
    ldconfig

# ========= 第三阶段：OpenVINO 安装 =========
FROM ceres_stage AS openvino_stage

ARG http_proxy
ARG https_proxy
ENV http_proxy=$http_proxy
ENV https_proxy=$https_proxy

# 下载并安装 OpenVINO
RUN mkdir -p /opt/intel && \
    cd /tmp && \
    curl -L --fail --silent --show-error \
    https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6/linux/l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f47d2335_x86_64.tgz \
    -o openvino.tgz && \
    tar -xzf openvino.tgz && \
    # 修复：将移动的目录名更正为 tar 解压出来的实际目录名
    mv l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f47d2335_x86_64 /opt/intel/openvino_2024.6.0 && \
    rm openvino.tgz

# 确保在正确的目录下执行依赖安装脚本
RUN yes | (cd /opt/intel/openvino_2024.6.0 && ./install_dependencies/install_openvino_dependencies.sh)

# 为方便使用，创建软链接
RUN cd /opt/intel && \
    ln -s openvino_2024.6.0 openvino_2024

ENV OPENVINO_DIR=/opt/intel/openvino_2024
ENV LD_LIBRARY_PATH="${OPENVINO_DIR}/runtime/lib/intel64:${LD_LIBRARY_PATH}"
ENV PATH="${OPENVINO_DIR}/tools:${PATH}"

# 更新动态链接库缓存
RUN echo 'source /opt/intel/openvino_2024/setupvars.sh' >> /etc/bash.bashrc && \
    ldconfig && \
    echo "${OPENVINO_DIR}/runtime/lib/intel64" >> /etc/ld.so.conf.d/openvino.conf && \
    ldconfig
# ========= 最终运行环境 =========
# 最终的镜像只提供环境，不包含源代码
FROM openvino_stage AS runtime

ARG http_proxy
ARG https_proxy
ENV http_proxy=$http_proxy
ENV https_proxy=$https_proxy

# 设置工作目录
WORKDIR /app

# 设置容器启动时要运行的默认命令为 bash，这样你可以进入容器后手动编译和运行
CMD ["bash"]
