# Copyright 2019 Mikael Arguedas
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

ARG FROM_IMAGE=osrf/ros2:devel
FROM $FROM_IMAGE

ARG ROS_DISTRO=dashing
ENV ROS_DISTRO=$ROS_DISTRO

# install building tools
RUN apt-get -qq update && \
    apt-get -qq upgrade -y && \
    apt-get -qq install ros-$ROS_DISTRO-ros-core -y && \
    rm -rf /var/lib/apt/lists/*

# setup colcon mixin / meta
RUN apt-get -qq update && \
    apt-get -qq install python3-colcon-mixin -y && \
    rm -rf /var/lib/apt/lists/*
RUN colcon mixin add upstream https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add origin https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

ENV TB3_DEMO_DIR $HOME/tb3_demo_roscon2019
# setup underlay
ENV TB3_UNDERLAY_WS $TB3_DEMO_DIR/tb3_underlay_ws
# copy optional additional_repos.repos
COPY ./dependencies.repos $TB3_UNDERLAY_WS/
RUN mkdir -p $TB3_UNDERLAY_WS/src
WORKDIR $TB3_UNDERLAY_WS
RUN vcs import src < dependencies.repos

# build underlay
RUN apt-get -qq update && rosdep install -y \
    --from-paths src \
    --ignore-src \
    --skip-keys "libopensplice69 rti-connext-dds-5.3.1" \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon \
    build \
    --merge-install \
    --mixin build-testing-off release \
    --cmake-args --no-warn-unused-cli

ENV ROS_PACKAGE_PATH=$TB3_UNDERLAY_WS/install/share:$ROS_PACKAGE_PATH

# setup overlay
ENV TB3_OVERLAY_WS $TB3_DEMO_DIR/tb3_overlay_ws
RUN mkdir -p $TB3_OVERLAY_WS/src
COPY ./turtlebot3.repos $TB3_OVERLAY_WS/
WORKDIR $TB3_OVERLAY_WS

RUN vcs import src < turtlebot3.repos
RUN apt-get -qq update && rosdep install -y \
    --from-paths src \
    --ignore-src \
    --skip-keys "turtlebot3_lidar libopensplice69 rti-connext-dds-5.3.1" \
    && rm -rf /var/lib/apt/lists/*

RUN . $TB3_UNDERLAY_WS/install/setup.sh && colcon \
    build \
    --merge-install \
    --mixin build-testing-off release \
    --cmake-args --no-warn-unused-cli

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
