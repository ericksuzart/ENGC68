FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# install gazebo
RUN apt update && \
    sudo apt install -y ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control && \
    apt update

# install nvidia-340 driver for the Macbook
RUN apt install -y software-properties-common && \
    sudo add-apt-repository ppa:kelebek333/nvidia-legacy && \
    apt update && \
    sudo apt install -y nvidia-340

RUN source ros_entrypoint.sh

RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

ARG USERNAME

ARG UUID
ARG UGID

RUN useradd -m $USERNAME && \
    echo "$USERNAME:$USERNAME" | chpasswd && \
    usermod --shell /bin/bash $USERNAME && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    usermod --uid $UUID $USERNAME && \
    groupmod --gid $UGID $USERNAME

COPY rootfs/ /

USER $USERNAME

RUN echo "export DISPLAY=\"`sed -n 's/nameserver //p' /etc/resolv.conf`:0\"" >> ~/.bashrc

USER root



