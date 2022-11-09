# me396p_project
meaningful description

## Setup
how to install Dcoker

## Run a container
If you are using operating system which uses X11 display server or Wayland with XWayland support, you can run the container with the following command:

```bash
./container/run.sh
```

The image of the container will be build during the first launch, so it will take more time to start up. After the launch process is completed, you will be provided with a command line interface. Graphical applications that you run will be displayed natively in your operating system. This approach was tested on Ubuntu 20.04 and 22.04, but should work on most of modern Linux systems.

If you are using Windows or Mac (or Linux system on pure Wayland), you can run container with a VNC interface and interact with it using your internet browser. Use the following command

```bash
./container/run.sh vnc
```

and navigate to

```
localhost:6080
```

in any browser window. You will see a desktop environment launched inside of a Docker container. Open any terminal.

## Launch a package
Navigate to ROS workspace

```bash
cd ./ros2_ws
```
and build the PADWQ package

```bash
colcon build
```

Then, source the package you have build

```bash
source install/setup.bash
```

and run it

```bash
ros2 launch padwq padwq.launch.py
```
