ARG FROM_IMAGE=osrf/ros2:nightly

# multi-stage for caching
FROM $FROM_IMAGE AS cache

# clone underlay source
ENV UNDERLAY_WS /opt/underlay_ws
RUN mkdir -p $UNDERLAY_WS/src
WORKDIR $UNDERLAY_WS
COPY ./install/underlay.repos ./
RUN vcs import src < underlay.repos

# copy overlay source
ENV OVERLAY_WS /opt/overlay_ws
RUN mkdir -p $OVERLAY_WS/src
WORKDIR $OVERLAY_WS
COPY ./install/overlay.repos ./
RUN vcs import src < overlay.repos

# copy manifests for caching
WORKDIR /opt
RUN find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp \
    && find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp

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

# copy underlay manifests
ENV UNDERLAY_WS /opt/underlay_ws
COPY --from=cache /tmp/underlay_ws $UNDERLAY_WS
WORKDIR $UNDERLAY_WS

# install underlay dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
      --skip-keys " \
        gazebo9 \
        libgazebo9-dev \
      " \
    && rm -rf /var/lib/apt/lists/*

# copy underlay source
COPY --from=cache $UNDERLAY_WS ./

# build underlay source
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



# # copy overlay manifests
# ENV OVERLAY_WS /opt/overlay_ws
# COPY --from=cache /tmp/overlay_ws $OVERLAY_WS
# WORKDIR $OVERLAY_WS

# # install overlay dependencies
# RUN . $UNDERLAY_WS/install/setup.sh && \
#     apt-get update && rosdep install -q -y \
#       --from-paths \
#         src \
#         $UNDERLAY_WS/src \
#       --ignore-src \
#     && rm -rf /var/lib/apt/lists/*

# # copy overlay source
# COPY --from=cache $OVERLAY_WS ./

# # build overlay source
# ARG OVERLAY_MIXINS="release ccache"
# RUN . $UNDERLAY_WS/install/setup.sh && \
#     colcon build \
#       --symlink-install \
#       --mixin $OVERLAY_MIXINS






# # clone overlay package repos
# ENV TB3_OVERLAY_WS /opt/tb3_overlay_ws
# RUN mkdir -p $TB3_OVERLAY_WS/src
# WORKDIR $TB3_OVERLAY_WS
# COPY .docker/overlay.repos ./
# RUN vcs import src < overlay.repos
# # Install extra sources from this repo
# COPY example_nodes/ src/example_nodes
# # RUN vcs import src < src/ros-planning/navigation2/tools/ros2_dependencies.repos

# # install overlay package dependencies
# RUN . /opt/ros/$ROS_DISTRO/setup.sh \
#     && rosdep update \
#     && rosdep install -y \
#       --from-paths src \
#       --ignore-src \
#       --skip-keys " \
#             ament_mypy \
#             libopensplice69 \
#             rti-connext-dds-5.3.1 \
#         " \
#     && rm -rf /var/lib/apt/lists/*

# # build overlay package source
# # RUN touch $TB3_OVERLAY_WS/src/turtlebot3/turtlebot3_node/COLCON_IGNORE
# RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
#     colcon build \
#       --symlink-install

# # generate artifacts for keystore
# ENV TB3_DEMO_DIR $TB3_OVERLAY_WS/..
# WORKDIR $TB3_DEMO_DIR
# COPY policies policies
# RUN . $TB3_OVERLAY_WS/install/setup.sh && \
#     ros2 security generate_artifacts -k keystore \
#       -p policies/tb3_gazebo_policy.xml \
#       -n /_ros2cli

# # copy demo files
# COPY maps maps
# COPY configs configs
# COPY .gazebo /root/.gazebo

# # source overlay workspace from entrypoint
# RUN sed --in-place \
#       's|^source .*|source "$TB3_OVERLAY_WS/install/setup.bash"|' \
#       /ros_entrypoint.sh && \
#     cp /etc/skel/.bashrc ~/ && \
#     echo 'source "$TB3_OVERLAY_WS/install/setup.bash"' >> ~/.bashrc

# ENV TURTLEBOT3_MODEL='burger' \
#     GAZEBO_MODEL_PATH=/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH
