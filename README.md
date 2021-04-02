# Secure Turtlebot3 Demo

This repository includes a demo for securing a simulated Turtlebot3 using SROS2; including sensor and control topics as well the relevant portions of the slam_toolbox and navigation2 software stacks.

## Setting the Demo

To run this demo using docker, the following dependencies are required:

* [ubuntu](https://ubuntu.com/)
  * Login in using x11 for simple display forwarding with containers.
  * Other linux distributions may work, but we'll focus on ubuntu.
* [docker](https://www.docker.com/)
  * Prior to workshop, please pull this large image ahead of time:
  * `docker pull rosswg/turtlebot3_demo:roscon19`
* [rocker](https://github.com/osrf/rocker)
  * Please ensure display forwarding is working with rocker.
  * [nvidia-docker](https://github.com/NVIDIA/nvidia-docker) is also useful for those with a GPU.
* [off-your-rocker](https://github.com/sloretz/off-your-rocker)
  * Rocker extension, for passing through arbitrary Docker arguments.

  For those who can't use linux containers or for detailed instructions on how to build, you may still follow the general build steps of the [Dockerfile](Dockerfile).

## Running the demo:

Start by attaching to a byobu session in a new container using rocker. [Byobu](http://www.byobu.org/) is simple wrapper for tmux, where the `F2` key creates and new window, and  `F3`/`F4` move focus among windows, and `Shift-<arrow-keys>` to move focus among window splits. To later exit the entire session, `F6` will exit from byobu and subsequently stop the container and all child processes. Use `Shift-F1` to displays all keybindings. Omit the `--nvidia` arg if you don't have dedicated GPU for hardware acceleration of 3D OpenGL views:

``` bash
rocker --x11 --nvidia rosswg/turtlebot3_demo:roscon19 "byobu -f configs/unsecure.conf attach"
```

![](media/mapping_start.png)

Byobu starts a new session and launch the turtlebot3 demo over several windows:

* `turtlebot`
  * turtlebot3 gazebo simulation
  * keyboard control teleportation
  * echo command velocity
* `navigation`
  * navigation2 planning stack
  * initialize pose script
  * navigation goal script
* `mapping`
  * slam_toolbox mapping stack
  * save map file
  * map topic info
* `sros`
  * tree view of keystore
  * generate artifacts command
  * list ROS environment variables
* `diagnostic`
  * glance overview of containerized processes

You can first drive the robot around and generate a map using the teleoperation script in the selected shell. You can go to the mapping window to save your map.

[![](media/mapping3.png)](media/mapping.mp4)

Feel free to poke around, open a new window and list or echo topics and services. You can explore the other panes as well, for example you can stop slam_toolbox in the `mapping` window and start the navigation launchfile from the `navigation` window. You will stat by localizing the robot by initializing the pose and then setting a navigation goal via the scripts in the respective window panes. This can also be done graphically via rviz.

[![](media/localize.png)](media/localize.mp4)

## Running the secure demo:

So far we've simply launched the turtlebot3 without using SROS2. To enable security, simply exit the previous byobu session and start a new one now using the secure config:

``` bash
rocker --x11 --nvidia rosswg/turtlebot3_demo:roscon19 "byobu -f configs/secure.conf attach"
```

This config is the exact same as the previous one, apart from a few SROS2 related environment variables that are exported:

``` bash
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_ROOT_DIRECTORY=$TB3_DEMO_DIR/keystore
export ROS_SECURITY_LOOKUP_TYPE=MATCH_PREFIX
```

These variables simply enable as well as enforce security for all ros2 nodes while specifying the lookup path/strategy for loading key and access control artifacts. For more details on what secure options exist and what each is for, please view the design doc here:

* https://design.ros2.org/articles/ros2_dds_security.html

## Re-generate security artifacts

Using the same attached session above, now lets try and generate security artifacts for ourselves, rather than simply using the same artifacts that came bundled in the demo docker image. We can start by switching over to the sros window and clearing out the existing keystore:

``` bash
cd $TB3_DEMO_DIR
rm -r keystore/*/
```

This will delete all security artifacts for every node, including their certificates and signed permission/governance files. However this will leave the same Certificate Authority (CA) files intact, so that we can play with secure networking later; otherwise new CAs will be generated, limiting later demonstrations.

Next we can halt all of the launch files and ros nodes running in the container, simply by switching the focus to the relevant pane and `Ctrl+C`ing the shell. You can switch to the diagnostic window to glance at all the processes in the container to check if you've missed any.

For now, lets only generate a profile for securing the communication between the teleop node and the simulation. Later we can test than unauthorized teleop nodes fail to connect once SROS is reestablished. Start by disabling security for the panes that were running the simulation and teleop nodes, and then starting them back up:


``` bash
export ROS_SECURITY_ENABLE=false
# <ros2 run or launch command>
```

Now switch over to the sros window and use the `generate_policy` argument with the security verb to generate a new profile. This will inspect current rosgraph to train a starter profile so that we don't have to write it by hand.

``` bash
ros2 security generate_policy policy/my_policy.xml
```

Before using the profile, its wise to audit it first to make sure it contains what we expect. We'll use [micro](https://github.com/zyedidia/micro) to view and edit the file. Simply use `Ctrl+S` to save and `Ctrl+Q` to quit the file, or press `Crtl+E` and type `help` for more info. Clicking/scrolling the mouse moves your cursor as expected.

``` bash
micro policy/my_policy.xml
```

We'll use this moment to tweak the policy and add a few pervasive dev permission so that ros2cli functions the same, however you should avoid resorting to wildcards when possible or when outside development. Simply copy and append this into `<profiles>` element:

``` xml
    <profile node="_ros2cli" ns="/">
      <services reply="ALLOW" request="ALLOW">
        <service>*</service>
      </services>
      <topics publish="ALLOW" subscribe="ALLOW">
        <topic>*</topic>
      </topics>
      <actions call="ALLOW" execute="ALLOW">
        <action>*</action>
      </actions>
    </profile>
```

Next we can repopulated the keystore for all profiles via:

``` bash
ros2 security generate_artifacts \
  -k keystore \
  -p policy/my_policy.xml \
  -n /_ros2cli
```

Finally, go back and re-enable security and restart simulation and the teleop nodes.

``` bash
export ROS_SECURITY_ENABLE=true
# <ros2 run or launch command>
```

Now try starting another teleop node with security disabled and check that only the secure teleop node can drive the robot.

## Developing
To rebuild this demo locally if you are working on it, you can rebuild the Docker image with the same tag, so all above demo commands will work correctly.

``` bash
git clone git@github.com:ros-swg/turtlebot3_demo.git
cd turtlebot3_demo
docker build --tag rosswg/turtlebot3_demo .
```

To attach the demo to the host's network interfaces, you may need to run the container as privileged to get the rviz and gazebo GUI to adapt with the dbus, while also using the host's PID and IPC namespace to avoid issues with colliding DDS GUIDs as referenced in this answers.ros.org question [here](https://answers.ros.org/question/296828/ros2-connectivity-across-docker-containers-via-host-driver)

```
rocker --x11 --nvidia --oyr-run-arg " \
    --ipc=host \
    --net=host \
    --pid=host \
    --privileged" \
  rosswg/turtlebot3_demo \
  "byobu -f configs/unsecure.conf attach"
```