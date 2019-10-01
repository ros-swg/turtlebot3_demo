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
This launchfile demonstrates resource limitations on sandboxed nodes.

It spins up a program that tries to fork 32 simultaneous processes that each take up as much
CPU time as possible.
However, due to the constraints put on this node by the sandbox, it is not able to significantly
disrupt the performance of the navigation demo.

For more details please see the launch_ros_sandbox project at
https://github.com/aws-robotics/launch-ros-sandbox
"""
import os

from launch import LaunchDescription

from launch_ros_sandbox.actions import SandboxedNodeContainer
from launch_ros_sandbox.descriptions import DockerPolicy
from launch_ros_sandbox.descriptions import SandboxedNode

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    node = SandboxedNode(
        package='example_nodes',
        node_executable='cpu_hog',
        arguments=['32'],
    )
    container = SandboxedNodeContainer(
        sandbox_name='bad_actor',
        policy=DockerPolicy(
            tag='roscon19',
            repository='rosswg/turtlebot3_demo',
            container_name='turtlebot3_navigation',
        ),
        node_descriptions=[node],
    )

    return LaunchDescription([container])
