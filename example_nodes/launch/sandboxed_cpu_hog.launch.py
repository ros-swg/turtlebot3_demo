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

For more information on configurable resource limits, see the `docker run` documentation
https://docs.docker.com/engine/reference/run/#runtime-constraints-on-resources
"""
from launch import LaunchDescription

from launch_ros_sandbox.actions import SandboxedNodeContainer
from launch_ros_sandbox.descriptions import DockerPolicy
from launch_ros_sandbox.descriptions import SandboxedNode


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(
        SandboxedNodeContainer(
            sandbox_name='my_sandbox',
            policy=DockerPolicy(
                tag='latest',
                repository='rosswg/turtlebot3_demo',
                container_name='sandboxed-cpu-hog',
                run_args={
                    # CPU quota, in microseconds per scheduler period.
                    # The default scheduler period is 100ms == 100000us
                    # Therefore the below value of 200000us limits this container to using,
                    # at most, 2 cpu cores' worth of processing
                    'cpu_period': 100000,
                    'cpu_quota': 200000,
                },
                entrypoint='/ros_entrypoint.sh',
            ),
            node_descriptions=[
                SandboxedNode(
                    package='example_nodes',
                    node_executable='cpu_hog',
                ),
            ]
        )
    )

    return ld


if __name__ == '__main__':
    import sys
    from launch import LaunchService
    ls = LaunchService(argv=sys.argv[1:], debug=True)
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
