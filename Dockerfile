ARG FROM_IMAGE=ros:dashing
FROM $FROM_IMAGE

# install CI dependencies	
RUN apt-get update && apt-get install -q -y \	
      ccache \	
    && rm -rf /var/lib/apt/lists/*

# clone underlay package repos
ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS
COPY ./docker/underlay.repos ./
RUN vcs import src < underlay.repos

# install underlay package dependencies
RUN apt-get update && \
    rosdep install -q -y \
      --from-paths \
        src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build underlay package source
ARG CMAKE_BUILD_TYPE=Release
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache

# clone overlay package repos
ENV TB3_WS /opt/tb3_ws
RUN mkdir -p $TB3_WS/src
WORKDIR $TB3_WS
COPY ./docker/overlay.repos ./
RUN vcs import src < overlay.repos

# install overlay package dependencies
RUN apt-get update && \
    rosdep install -q -y \
      --from-paths \
        $ROS_WS/src \
        src \
      --ignore-src \
        --skip-keys "\
            turtlebot3_lidar" \
    && rm -rf /var/lib/apt/lists/*

# build overlay package source
RUN . $ROS_WS/install/setup.sh && \
     colcon build \
       --symlink-install \
       --cmake-args \
         -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE \
         -DCMAKE_C_COMPILER_LAUNCHER=ccache \
         -DCMAKE_CXX_COMPILER_LAUNCHER=ccache

# source overlay workspace from entrypoint
RUN sed --in-place \
      's|^source .*|source "$TB3_WS/install/setup.bash"|' \
      /ros_entrypoint.sh

WORKDIR $TB3_WS
COPY ./maps ./policies ./
ENV TURTLEBOT3_MODEL burger
ENV TB3_POLICY_FILE $TB3_WS/policies/tb3_gazebo_policy.xml