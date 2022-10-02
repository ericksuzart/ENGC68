# Docker based ROS repository

This repository contains a set of various workspaces that can be used to build a docker image with ROS and some additional packages. Every workspace has its own ros version, packages and dependencies. All the dependecies are installed in the docker image, so you can use the workspaces without installing anything beside docker on your host machine.

## Building the image

To build the docker image go to the workspace folder and run the following script:

``` bash
    ./build.sh
```

This will build the image and tag it as `ros1:ENGC68-<workspace_folder_name>`.

## Running the image

To run the image, go to the workspace folder and run the following script:

``` bash
    ./run_container.sh
```

This will run the image and mount the source directory as a volume in the container. This means that any changes made to the files in the source directory will be reflected in the container.  
