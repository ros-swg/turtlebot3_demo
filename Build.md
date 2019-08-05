# Instructions to build tb3_demo

You can also use the Dockerfile at [dockerfile](docker/Dockerfile)

## Prerequisites

Having [ROS 2 Dashing](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/) installed on your system and sourced in your terminal
This demo and instructions have been tested only on Ubuntu Bionic

Make sure rosdep is installed and up to date
```bash
sudo apt -qq update && sudo apt -qq -y install python-rosdep && python3-vcstool
sudo rosdep init
rosdep update
```

## Setup the environment

```bash
source /opt/ros/dashing/setup.bash
export TB3_DEMO_DIR=$HOME/tb3_demo_roscon2019
mkdir $TB3_DEMO_DIR && cd $TB3_DEMO_DIR
git clone https://github.com/mikaelarguedas/tb3_demo.git
export TB3_UNDERLAY_WS=$TB3_DEMO_DIR/tb3_underlay_ws
mkdir -p $TB3_UNDERLAY_WS/src
```

## Install and build dependencies

### Install the dependencies

```bash
cp $TB3_DEMO_DIR/tb3_demo/docker/dependencies.repos $TB3_UNDERLAY_WS/
cd $TB3_UNDERLAY_WS/
vcs import src < dependencies.repos 
sudo apt-get -qq update && rosdep install -y     --from-paths src     --ignore-src --skip-keys "libopensplice69 rti-connext-dds-5.3.1"
```

### Build the workspace

```bash
. /opt/ros/dashing/setup.sh && colcon     build     --merge-install     --mixin build-testing-off release     --cmake-args --no-warn-unused-cli
```

## Get and build turtlebot3 packages

```bash
export ROS_PACKAGE_PATH=$TB3_UNDERLAY_WS/install/share:$ROS_PACKAGE_PATH
export TB3_OVERLAY_WS=$TB3_DEMO_DIR/tb3_overlay_ws 
mkdir -p $TB3_OVERLAY_WS/src
cp $TB3_DEMO_DIR/tb3_demo/docker/turtlebot3.repos $TB3_OVERLAY_WS/
cd $TB3_OVERLAY_WS
vcs import src < turtlebot3.repos 
rosdep install -y     --from-paths src     --ignore-src     --skip-keys "turtlebot3_lidar libopensplice69 rti-connext-dds-5.3.1"
. $TB3_UNDERLAY_WS/install/setup.sh && colcon     build     --merge-install     --mixin build-testing-off release     --cmake-args --no-warn-unused-cli
```

## Set the variables in the bashrc for future use

You can add the following environment variables to your .bashrc to save time when running the demo

```bash
export TB3_DEMO_DIR=$HOME/tb3_demo_roscon2019
export TB3_UNDERLAY_WS=$TB3_DEMO_DIR/tb3_underlay_ws
export TB3_OVERLAY_WS=$TB3_DEMO_DIR/tb3_overlay_ws 
export TURTLEBOT3_MODEL=burger
export TB3_DEMO_POLICY_FILE="$TB3_DEMO_DIR/tb3_demo/policies/tb3_gazebo_policy.xml"
source $TB3_OVERLAY_WS/install/setup.bash
```
