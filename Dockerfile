FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

# install basic tools
RUN apt update && \
    sudo apt install -y \
        wget \
        git \
        python3-pip \
        locales \
        software-properties-common && \
    sudo pip3 install -U catkin_tools && \
    apt update

RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# install ROS packages
RUN sudo apt install -y \
        ros-noetic-husky-desktop \
        ros-noetic-husky-simulator \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-gazebo-ros-control

ENV DEBIAN_FRONTEND=noninteractive

RUN mkdir -p /workspace/src && \
    cd /workspace/src && \
    git clone -b noetic https://github.com/ericksuzart/lar_gazebo.git && \
    source /opt/ros/noetic/setup.bash && \
    cd /workspace && \
    catkin build

# install gpu driver
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration && \
    sudo add-apt-repository ppa:kelebek333/nvidia-legacy && \
    sudo apt install -y \
        nvidia-340 

WORKDIR /workspace

# RUN useradd -ms /bin/bash ros
# USER ros

# ARG UUID="$(id -u)"
# ARG UGID="$(id -g)"

# RUN useradd -m $USERNAME && \
#     echo "$USERNAME:$USERNAME" | chpasswd && \
#     usermod --shell /bin/bash $USERNAME && \
#     usermod -aG sudo $USERNAME && \
#     echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
#     chmod 0440 /etc/sudoers.d/$USERNAME && \
#     usermod --uid $UUID $USERNAME && \
#     groupmod --gid $UGID $USERNAME

# RUN echo "export DISPLAY=\"`sed -n 's/nameserver //p' /etc/resolv.conf`:0\"" >> ~/.bashrc

COPY run_container.sh entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["/bin/bash"]

