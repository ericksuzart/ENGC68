# Docker based ROS repository

This repository contains a Dockerfile that can be used to build a Docker image that contains the necessary software to run the projects in the course ENGC68. Therefore, you will need to install Docker on your computer.

## Building the image

To build the image, run the following command:

    ./build.sh

This will build the image and tag it as `ros1:topicos-atvd`.

## Running the image

To run the image, run the following command:

    ./run_container.sh

This will run the image and mount the source directory as a volume in the container. This means that any changes made to the files in the source directory will be reflected in the container.
