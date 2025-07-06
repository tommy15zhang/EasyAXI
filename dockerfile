FROM ubuntu:22.04

# non interactive mode to avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    curl \
    wget \
    python3 \
    python3-pip \
    python3-setuptools \
    python3-venv \
    make \
    cmake \
    libgoogle-perftools-dev \
    verilator \
    iverilog \
    && apt-get clean

# Install Python packages
RUN pip3 install cocotb

# workspace directory
WORKDIR /workspace

# 默认进入 bash
CMD ["/bin/bash"]