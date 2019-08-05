# tb3_demo
Repository to build and test Turtlebot3 packages


For instructions on how to build, please follow the building steps of the [dockerfile](docker/Dockerfile)

## Running the demo:

The following environment variables will be used in the instructions.
You can set them in your bashrc to save some time or define them in theterminal used for the demo

```bash
export TB3_DEMO_DIR=$HOME/tb3_demo_roscon2019
export TB3_UNDERLAY_WS=$TB3_DEMO_DIR/tb3_underlay_ws
export TB3_OVERLAY_WS=$TB3_DEMO_DIR/tb3_overlay_ws
export TURTLEBOT3_MODEL=burger
export TB3_DEMO_POLICY_FILE="$TB3_DEMO_DIR/tb3_demo/policies/tb3_gazebo_policy.xml"
```
You'll also need to source the workspace in each terminal:
```bash
source $TB3_OVERLAY_WS/install/setup.bash
```

### Launch the Gazebo TB3 simulation

Shell 1:
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$TB3_OVERLAY_WS/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Drive the robot around

Shell 2:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
### Running cartographer

#### Launch the cartographer node

Shell 3:

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```

This will launch RViz displaying the state of the map being built. go back in Shell2 and drive the robot through the environment until the map is complete.

#### Save the map

Shell 4:

```bash
ros2 run nav2_map_server map_saver -f $TB3_DEMO_DIR/tb3_demo/maps/map
```

### Navigating a known map

Now we're done driving by hand and will try to use navigation algorithms instead!
You can shut down terminals 2, 3 and 4 and just keep the gazebo simulation running (shell 1)

#### Launching the navigation stack

Shell 4:
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$TB3_DEMO_DIR/tb3_demo/maps/map_original.yaml
```

This will open RViz with the loaded map displayed.

#### Setting an initial pose

Before being able to navigate: an Initial pose needs to be provided to the robot.
To do this in RViz, click on the "2D Pose Estimate" in the tool bar, and then draw an arrow matching the current robot position in the world.

#### Send a navigation goal
We are now ready to navigate our robot through the world!
Set a navigation goal using the "2D Nav Goal" button in RViz's toolbar

### Running the demo with security enabled

#### Generate security artifacts for the application

First we will generate all the security artifacts required for our nodes to run using DDS-Security
*TODO* replace this long node list by an invocation of `generate_artifacts` using only a policy file
All the artifacts will be placed in `$TB3_OVERLAY_WS/keystore`


```bash
source $TB3_OVERLAY_WS/install/setup.bash
cd $TB3_DEMO_DIR
export TB3_DEMO_POLICY_FILE="$TB3_DEMO_DIR/policies/tb3_gazebo_policy.xml"
ros2 security generate_artifacts -k keystore \
  -n \
    /_client_node \
  -p $TB3_DEMO_POLICY_FILE
```

#### Setup the environment to use security

To notify ROS 2 that we want ot use DDS-Security, we use environment variables
Run these commands in shell 1 and 4 for the navigation demo.
```bash
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_ROOT_DIRECTORY=$TB3_DEMO_DIR/keystore
export ROS_SECURITY_LOOKUP_TYPE=MATCH_PREFIX
```

Then refer to steps described in "Navigating a known map"
