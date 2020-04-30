ARG FROM_IMAGE=osrf/ros2:nightly
ARG UNDERLAY_WS=/opt/ros/underlay_ws
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cache

# clone underlay source
ARG UNDERLAY_WS
WORKDIR $UNDERLAY_WS/src
COPY ./install/underlay.repos ../
RUN vcs import ./ < ../underlay.repos && \
    find ./ -name ".git" | xargs rm -rf

# copy overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./install/overlay.repos ../
RUN vcs import ./ < ../overlay.repos && \
    find ./ -name ".git" | xargs rm -rf

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS build

# install helpful developer tools
RUN apt-get update && apt-get install -y \
      bash-completion \
      byobu \
      ccache \
      fish \
      glances \
      micro \
      nano \
      python3-argcomplete \
      tree \
      vim \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
# setup sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list
# install gazebo packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
      libgazebo11-dev \
    && rm -rf /var/lib/apt/lists/*

# install underlay dependencies
ARG UNDERLAY_WS
WORKDIR $UNDERLAY_WS
COPY --from=cache /tmp/$UNDERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
      --skip-keys " \
        gazebo11 \
        libgazebo11-dev \
      " \
    && rm -rf /var/lib/apt/lists/*

# build underlay source
COPY --from=cache $UNDERLAY_WS/src ./src
ARG UNDERLAY_MIXINS="ccache release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $UNDERLAY_MIXINS \
      --cmake-args \
        --no-warn-unused-cli \
        -DCMAKE_CXX_FLAGS=" \
          -Wno-deprecated-declarations \
        "

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cache /tmp/$OVERLAY_WS/src ./src
RUN . $UNDERLAY_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths \
        src \
        $UNDERLAY_WS/src \
      --ignore-src \
      --skip-keys " \
        gazebo11 \
        libgazebo11-dev \
        cartographer_ros \
        hls_lfcd_lds_driver \
        dynamixel_sdk \
      " \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cache $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release ccache"
RUN . $UNDERLAY_WS/install/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $OVERLAY_MIXINS \
      --packages-up-to \
        "turtlebot3_simulations" \
        "turtlebot3_navigation2" \
        "turtlebot3_teleop" \
      --packages-skip \
        "turtlebot3_node" \
        "turtlebot3"

# generate artifacts for keystore
ENV TB3_DEMO_DIR $OVERLAY_WS/..
WORKDIR $TB3_DEMO_DIR
COPY policies policies
# RUN . $OVERLAY_WS/install/setup.sh && \
#     ros2 security generate_artifacts -k keystore \
#       -p policies/tb3_gazebo_policy.xml \
#       -n /_ros2cli

# copy demo files
COPY maps maps
COPY configs configs
COPY .gazebo /root/.gazebo

# source overlay workspace from entrypoint
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place \
      's|^source .*|source "$OVERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh && \
    cp /etc/skel/.bashrc ~/ && \
    echo 'source "$OVERLAY_WS/install/setup.bash"' >> ~/.bashrc

ENV TURTLEBOT3_MODEL='burger' \
    GAZEBO_MODEL_PATH=$OVERLAY_WS/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH
