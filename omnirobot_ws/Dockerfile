FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

LABEL app="topicos" \
      version="omnibot" \
      description="Imagem da atividade de desenvolvimento de um robo omnidirecional" \
      maintainer="Erick Suzart Souza"

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

ENV DEBIAN_FRONTEND=noninteractive

# install gpu driver
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration && \
    sudo add-apt-repository ppa:kelebek333/nvidia-legacy && \
    sudo apt install -y \
        nvidia-340 

# install ROS packages
RUN sudo apt install -y \
        ros-$ROS_DISTRO-gazebo-* \
        ros-$ROS_DISTRO-desktop-full \
        ros*controller* \
        tree

# Modify the gazebo kill timeout
RUN sed -i 's/DEFAULT_TIMEOUT_SIGTERM = 2.0 #seconds/DEFAULT_TIMEOUT_SIGTERM = 1.0 #seconds/g' /opt/ros/$ROS_DISTRO/lib/python3/dist-packages/roslaunch/nodeprocess.py && \
    sed -i 's/DEFAULT_TIMEOUT_SIGINT  = 15.0 #seconds/DEFAULT_TIMEOUT_SIGINT  = 1.0 #seconds/g' /opt/ros/$ROS_DISTRO/lib/python3/dist-packages/roslaunch/nodeprocess.py

RUN mkdir -p /workspace/src

COPY src /workspace/src

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd /workspace && \
    catkin build

WORKDIR /workspace

# nvidia-docker hooks
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

COPY entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["/bin/bash"]

