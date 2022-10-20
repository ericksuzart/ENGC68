FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

LABEL app="topicos" \
      version="ATVD-3" \
      description="Imagem para testes em simulacao" \
      maintainer="Erick Suzart Souza"

# install basic tools
RUN apt update && \
    apt install -y \
        wget \
        git \
        python3-pip \
        locales \
        software-properties-common \
        locate \
        apt-utils \
        tree

# install catkin tools package
RUN sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list' && \
    wget http://packages.ros.org/ros.key -O - | apt-key add - && \
    apt update && \
    apt install -y python3-catkin-tools && \
    apt update

RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

ENV DEBIAN_FRONTEND=noninteractive

# install ROS packages
RUN apt install -y \
        ros-$ROS_DISTRO-gazebo-* 

# Modify the gazebo kill timeout
RUN sed -i 's/DEFAULT_TIMEOUT_SIGTERM = 2.0 #seconds/DEFAULT_TIMEOUT_SIGTERM = 1.0 #seconds/g' /opt/ros/$ROS_DISTRO/lib/python3/dist-packages/roslaunch/nodeprocess.py && \
    sed -i 's/DEFAULT_TIMEOUT_SIGINT  = 15.0 #seconds/DEFAULT_TIMEOUT_SIGINT  = 1.0 #seconds/g' /opt/ros/$ROS_DISTRO/lib/python3/dist-packages/roslaunch/nodeprocess.py

RUN mkdir -p /workspace/src

COPY src /workspace/src

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd /workspace && \
    rosdep update && \
    rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src -y && \
    catkin build

WORKDIR /workspace

# install gpu driver
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration && \
    sudo add-apt-repository ppa:kelebek333/nvidia-legacy && \
    sudo apt install -y \
        nvidia-340 

COPY entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["/bin/bash"]
