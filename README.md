# tb3_demo
Repository to build and test Turtlebot3 packages


For instructions on how to build, please follow the building steps of the [dockerfile](docker/Dockerfile)

## Running the demo:

### Setup the environment
Shell 1:
```bash
# setup tb3 workspace
source <OVERLAY_WORKSPACE>/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<OVERLAY_WORKSPACE>/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
export TURTLEBOT3_MODEL=burger
```

### Launch the Gazebo TB3 simulation
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Drive the robot around

Shell 2:
```
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```
### Running cartographer

#### Launch the cartographer node

Shell 3:

```
# setup tb3 workspace
source <OVERLAY_WORKSPACE>/install/setup.bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

This will launch RViz displaying the state of the map being built. go back in Shell2 and drive the robot through the environment until the map is complete.

#### Save the map

Shell 4:
```
# setup tb3 workspace
source <OVERLAY_WORKSPACE>/install/setup.bash
ros2 run nav2_map_server map_saver -f <OVERLAY_WORKSPACE>/map
```

### Navigating a known map

#### Launching the navigation stack

Shell 4:
```
# setup tb3 workspace
source <OVERLAY_WORKSPACE>/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true
```

This will open RViz with the loaded map displayed.

#### Setting an initial pose

Before being able to navigate: an Initial pose needs to be provided to the robot.
To do this in RViz, click on the "2D Pose Estimate" in the tool bar, and then draw an arrow matching the current robot position in the world.

#### Send a navigation goal
We are now ready to navigate our robot through the world!
Set a navigation goal using the "2D Nav Goal" button in RViz's toolbar
