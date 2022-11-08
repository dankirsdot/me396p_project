ARG PARENT_IMAGE
FROM $PARENT_IMAGE

ARG ROS_DISTRO
RUN apt-get update \
    && apt-get install -y \
            ros-${ROS_DISTRO}-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*
