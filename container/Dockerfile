ARG PARENT_IMAGE
FROM $PARENT_IMAGE

ARG ROS_DISTRO
RUN apt-get update \
    && apt-get install -y \
            ros-${ROS_DISTRO}-desktop \
            ros-${ROS_DISTRO}-xacro \
            ros-${ROS_DISTRO}-joint-state-publisher-gui \
            ros-${ROS_DISTRO}-ros2-control \
            ros-${ROS_DISTRO}-ros2-controllers \
            ros-${ROS_DISTRO}-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*
