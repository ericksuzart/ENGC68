FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

LABEL app="topicos" \
      version="2" \
      description="Imagem para a disciplina ENGC68" \
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
        ros-noetic-husky-desktop \
        ros-noetic-husky-simulator \
        ros-noetic-gazebo-* \
        ros-noetic-desktop-full \
        ros*controller* \
        tree

RUN mkdir -p /workspace/src

COPY src/packages /workspace/src

RUN cd /workspace/src && \
    git clone -b noetic https://github.com/ericksuzart/lar_gazebo.git && \
    source /opt/ros/noetic/setup.bash && \
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

