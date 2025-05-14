# Use Ubuntu 24.04 LTS as the base image
FROM ubuntu:24.04

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
    apt-utils \
    bison \
    ca-certificates \
    ccache \
    check \
    curl \
    dos2unix \
    flex \
    git \
    gperf \
    lcov \
    libffi-dev \
    libncurses-dev \
    libssl-dev \
    libusb-1.0-0-dev \
    locales \
    make \
    ninja-build \
    python3 \
    python3-pip \
    python3-setuptools \
    unzip \
    wget \
    xz-utils \
    zip \
    && apt-get clean && rm -rf /var/lib/apt/lists/* && \
    update-alternatives --install /usr/bin/python python /usr/bin/python3 10

# Set locale to en_US.UTF-8
RUN locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

# Create non-root user for VS Code container access
ARG USERNAME=vscode
RUN useradd -ms /bin/bash $USERNAME

# Set working directory
WORKDIR /home/$USERNAME

# Set path variable
ENV PATH="/home/$USERNAME/.local/bin:/opt/esp/xtensa-esp32-elf/bin:${PATH}"

# ESP-IDF Toolchain Setup (run as root)
RUN mkdir -p /opt/esp && \
    wget -O /tmp/xtensa-esp32-elf.tar.gz https://dl.espressif.com/dl/xtensa-esp32-elf-linux64-1.22.0-97-gc752ad5-5.2.0.tar.gz && \
    tar -xzf /tmp/xtensa-esp32-elf.tar.gz -C /opt/esp && \
    rm /tmp/xtensa-esp32-elf.tar.gz

# Clone the ESP-IDF fork
RUN git clone https://github.com/openvehicles/esp-idf.git /opt/esp-idf && \
    cd /opt/esp-idf && \
    git submodule update --init --recursive

# Change ownership of ESP directories to non-root user
RUN chown -R $USERNAME:$USERNAME /opt/esp /opt/esp-idf

# Set IDF_PATH globally for all users
RUN echo 'export IDF_PATH=/opt/esp-idf' > /etc/profile.d/export_idf_path.sh && \
    chmod +x /etc/profile.d/export_idf_path.sh

# Switch to non-root user
USER $USERNAME

# Configure git safe directory
RUN git config --global --add safe.directory /opt/esp-idf

# Install Python dependencies
RUN pip3 install --break-system-packages -r /opt/esp-idf/requirements.txt
