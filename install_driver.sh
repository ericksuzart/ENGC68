#!/bin/bash

# install nvidia-340 driver for the Macbook
export DEBIAN_FRONTEND=noninteractive
DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration
apt install -y software-properties-common
sudo add-apt-repository ppa:kelebek333/nvidia-legacy
sudo apt install -y nvidia-340
