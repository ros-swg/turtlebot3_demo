ARG FROM_IMAGE=rosplanning/navigation2:master.release
FROM $FROM_IMAGE

ENV ROS_PACKAGE_PATH=/opt/ros/$ROS_DISTRO/share:$ROS_PACKAGE_PATH
ENV ROS_PACKAGE_PATH=$UNDERLAY_WS/install:$ROS_PACKAGE_PATH
ENV ROS_PACKAGE_PATH=$OVERLAY_WS/install:$ROS_PACKAGE_PATH

# clone underlay package repos
ENV TB3_UNDERLAY_WS /opt/tb3_underlay_ws
RUN mkdir -p $TB3_UNDERLAY_WS/src
WORKDIR $TB3_UNDERLAY_WS
COPY ./docker/underlay.repos ./
RUN vcs import src < underlay.repos
# RUN vcs import src < src/ros-planning/navigation2/tools/ros2_dependencies.repos

# install underlay package dependencies
RUN . $OVERLAY_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths \
        src \
      --ignore-src \
        --skip-keys "\
            libopensplice69 \
            rti-connext-dds-5.3.1" \
    && rm -rf /var/lib/apt/lists/*

# build underlay package source
ARG CMAKE_BUILD_TYPE=Release
RUN . $OVERLAY_WS/install/setup.sh && \
    colcon build \
      --symlink-install \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE
ENV ROS_PACKAGE_PATH=$TB3_UNDERLAY_WS/install:$ROS_PACKAGE_PATH

# clone overlay package repos
ENV TB3_OVERLAY_WS /opt/tb3_overlay_ws
RUN mkdir -p $TB3_OVERLAY_WS/src
WORKDIR $TB3_OVERLAY_WS
COPY ./docker/overlay.repos ./
RUN vcs import src < overlay.repos

# install overlay package dependencies
RUN . $TB3_UNDERLAY_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths \
        $TB3_OVERLAY_WS/src \
        $TB3_UNDERLAY_WS/src \
        $OVERLAY_WS/install \
        $UNDERLAY_WS/install \
      --ignore-src \
        --skip-keys "\
            dynamixel_sdk \
            hls_lfcd_lds_driver \
            turtlebot3_lidar \
            libopensplice69 \
            rti-connext-dds-5.3.1" \
    && rm -rf /var/lib/apt/lists/*

# build overlay package source
RUN touch $TB3_OVERLAY_WS/src/turtlebot3/turtlebot3_node/COLCON_IGNORE
RUN . $TB3_UNDERLAY_WS/install/setup.sh && \
     colcon build \
       --symlink-install \
       --cmake-args \
         -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE
ENV ROS_PACKAGE_PATH=$TB3_OVERLAY_WS/install:$ROS_PACKAGE_PATH

# source overlay workspace from entrypoint
RUN sed --in-place \
      's|^source .*|source "$TB3_OVERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh

WORKDIR $TB3_OVERLAY_WS
COPY ./maps ./policies ./
ENV TURTLEBOT3_MODEL burger
ENV TB3_POLICY_FILE $TB3_OVERLAY_WS/policies/tb3_gazebo_policy.xml