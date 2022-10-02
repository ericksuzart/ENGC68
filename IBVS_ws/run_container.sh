#!/bin/bash
XAUTH=/tmp/.docker.xauth
xhost +local:docker

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo "Running docker..."

BASEDIR=$(pwd -P)

# Define Docker volumes and environment variables
DOCKER_VOLUMES="
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --volume="/workspace" \
    --volume="${BASEDIR}/src":"/workspace/src":rw \
"

DOCKER_ENV_VARS="
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
"

DOCKER_CONFIG="
    --net=host \
    --ipc=host \
    --privileged \
    --user=$USERNAME \
"

DOCKER_ARGS=${DOCKER_VOLUMES}" "${DOCKER_ENV_VARS}" "${DOCKER_CONFIG}

docker run -it \
    ${DOCKER_ARGS} \
    ros1:topicos-IBVS \
    bash

echo "Finished."
