# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This launch file is meant to be used to showcase the launch_ros_sandbox
package capabilities.

When running the Turtlebot 3 security demo, the navigation nodes are sandboxed,
i.e., running in a docker container. Using the launch_ros_sandbox 
SandboxedNodeContainer and SandboxedNode here, we can replace the following
launch files:

    ros-swg/turtlebot3_demo/navigation2.launch.py
    ros-planning/navigation2/nav2_bringup_launch.py
    ros-planning/navigation2/nav2_localization_launch.py
    ros-planning/navigation2/nav2_navigation_launch.py

For more details please see the launch_ros_sandbox project at
https://github.com/aws-robotics/launch-ros-sandbox
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from nav2_common.launch import RewrittenYaml

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node

from launch_ros_sandbox.actions import SandboxedNodeContainer
from launch_ros_sandbox.descriptions import DockerPolicy
from launch_ros_sandbox.descriptions import SandboxedNode

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    # Get the launch directory
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    map_subscribe_transient_local = LaunchConfiguration(
        'map_subscribe_transient_local')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'bt_xml_filename': bt_xml_file,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local}

    configured_params = RewrittenYaml(
        source_file=params_file,
        rewrites=param_substitutions,
        convert_types=True)

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'map.yaml'))

    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            param_file_name))

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'nav2_default_view.rviz')

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),


        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'bt_xml_file',
            default_value=os.path.join(get_package_prefix('nav2_bt_navigator'),
                                       'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'use_lifecycle_mgr', default_value='true',
            description='Whether to launch the lifecycle manager'),

        DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='false',
            description='Whether to set the map subscriber QoS to transient local'),

        # The following defines a list of SandboxedNodes running in a docker container
        SandboxedNodeContainer(
            sandbox_name='security_sandbox',
            policy=DockerPolicy(
                tag='roscon19',
                repository='rosswg/turtlebot3_demo',
                container_name='turtlebot3_navigation',
            ),
            node_descriptions=[
                SandboxedNode(
                    package='nav2_lifecycle_manager',
                    node_executable='lifecycle_manager',
                    node_name='lifecycle_manager',
                    # output='screen',
                    parameters=[{'use_sim_time': use_sim_time},
                                {'autostart': autostart},
                                {'node_names': ['map_server',
                                                'amcl',
                                                'controller_server',
                                                'planner_server',
                                                'bt_navigator']}]),
                SandboxedNode(
                    package='nav2_map_server',
                    node_executable='map_server',
                    node_name='map_server',
                    # output='screen',
                    parameters=[configured_params]),

                SandboxedNode(
                    package='nav2_amcl',
                    node_executable='amcl',
                    node_name='amcl',
                    # output='screen',
                    parameters=[configured_params]),

                SandboxedNode(
                    package='nav2_lifecycle_manager',
                    node_executable='lifecycle_manager',
                    node_name='lifecycle_manager_localization',
                    # output='screen',
                    parameters=[{'use_sim_time': use_sim_time},
                                {'autostart': autostart},
                                {'node_names': ['map_server', 'amcl']}]),

                SandboxedNode(
                    package='nav2_controller',
                    node_executable='controller_server',
                    # output='screen',
                    parameters=[configured_params]),

                SandboxedNode(
                    package='nav2_planner',
                    node_executable='planner_server',
                    node_name='planner_server',
                    # output='screen',
                    parameters=[configured_params]),

                SandboxedNode(
                    package='nav2_recoveries',
                    node_executable='recoveries_node',
                    node_name='recoveries',
                    # output='screen',
                    parameters=[{'use_sim_time': use_sim_time}]),

                SandboxedNode(
                    package='nav2_bt_navigator',
                    node_executable='bt_navigator',
                    node_name='bt_navigator',
                    # output='screen',
                    parameters=[configured_params]),

                SandboxedNode(
                    package='nav2_lifecycle_manager',
                    node_executable='lifecycle_manager',
                    node_name='lifecycle_manager_navigation',
                    # output='screen',
                    parameters=[{'use_sim_time': use_sim_time},
                                {'autostart': autostart},
                                {'node_names': ['controller_server',
                                                'navfn_planner',
                                                'bt_navigator']}]),
            ]
        ),

        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            # output='screen'
        ),

    ])
