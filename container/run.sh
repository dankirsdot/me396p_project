#!/bin/bash

DISPLAY_SERVER=${1:-"x11"}

ROS_DISTRO=humble
IMAGE_NAME="me396p/ros:$ROS_DISTRO"

SCRIPT_PATH=$(readlink -f $BASH_SOURCE)
CONTAINER_PATH="${SCRIPT_PATH%/*}/"
WORKSPACE_PATH=${CONTAINER_PATH/container/ros2_ws}

# check if docker or podman are available in the system
if [ -x "$(command -v docker)" ]; then
    CONTAINER_ENGINE=docker
elif [ -x "$(command -v podman)" ]; then
    CONTAINER_ENGINE=podman
else
    echo "There is no container engine!"
    exit 0
fi

if [ $DISPLAY_SERVER == "x11" ]; then
    #set parent image
    PARENT_IMAGE="docker.io/ros:$ROS_DISTRO"
    
    # build an image
    $CONTAINER_ENGINE build -t $IMAGE_NAME $CONTAINER_PATH \
        --build-arg PARENT_IMAGE=$PARENT_IMAGE \
        --build-arg ROS_DISTRO=$ROS_DISTRO

    # run it
    xhost +local:docker
    DISPLAY=:1.0
    $CONTAINER_ENGINE run -it \
                        --net=host \
                        --device /dev/dri/ \
                        -e DISPLAY=$DISPLAY \
                        -v $HOME/.Xauthority:/root/.Xauthority:ro \
                        -v $WORKSPACE_PATH:/ros2_ws \
                        $IMAGE_NAME
elif [ $DISPLAY_SERVER == "wayland" ]; then
    # not working now: https://github.com/ros2/rviz/issues/847 
    #set parent image
    PARENT_IMAGE="docker.io/ros:$ROS_DISTRO"
    
    # build an image
    $CONTAINER_ENGINE build -t $IMAGE_NAME $CONTAINER_PATH \
        --build-arg PARENT_IMAGE=$PARENT_IMAGE \
        --build-arg ROS_DISTRO=$ROS_DISTRO

    $CONTAINER_ENGINE run -it \
                        -e XDG_RUNTIME_DIR=/tmp \
                        -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
                        -e GDK_BACKEND=wayland \
                        -e QT_QPA_PLATFORM=wayland \
                        -e CLUTTER_BACKEND=wayland \
                        -e SDL_VIDEODRIVER=wayland \
                        -v $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/tmp/$WAYLAND_DISPLAY  \
                        --net=host \
                        --device /dev/dri/ \
                        -v $WORKSPACE_PATH:/ros2_ws \
                        $IMAGE_NAME
elif [ $DISPLAY_SERVER == "vnc" ]; then
    # set parent image
    PARENT_IMAGE="docker.io/tiryoh/ros2-desktop-vnc:$ROS_DISTRO"
    
    # build an image
    $CONTAINER_ENGINE build -t $IMAGE_NAME $CONTAINER_PATH \
        --build-arg PARENT_IMAGE=$PARENT_IMAGE \
        --build-arg ROS_DISTRO=$ROS_DISTRO
    
    # run it
    $CONTAINER_ENGINE run -p 6080:80 \
                        --shm-size=512m \
                        --security-opt seccomp=unconfined \
                        -v $WORKSPACE_PATH:/home/ubuntu/ros2_ws \
                        $IMAGE_NAME
else
    echo "DISPLAY_SERVER! display server is not available!"
fi
