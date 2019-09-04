FROM ros:dashing

# patch rosdep for ament
RUN rosdep update && \
    apt-get purge python3-rosdep -y && \
    pip3 install git+https://github.com/ruffsl/rosdep.git@ament

# clone overlay package repos
ENV TB3_OVERLAY_WS /opt/tb3_overlay_ws
RUN mkdir -p $TB3_OVERLAY_WS/src
WORKDIR $TB3_OVERLAY_WS
COPY ./docker/overlay.repos ./
RUN vcs import src < overlay.repos

# install overlay package dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths \
        src \
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
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install

# install helpful developer tools
RUN apt-get update && apt-get install -y \
      bash-completion \
      byobu \
      glances \
      nano \
      python3-argcomplete \
      tmux \
      tree \
    && rm -rf /var/lib/apt/lists/*

# generate artifacts for keystore
WORKDIR $TB3_OVERLAY_WS
COPY ./maps ./maps
COPY ./policies ./policies
RUN . $TB3_OVERLAY_WS/install/setup.sh && \
    ros2 security generate_artifacts -k keystore \
      -p policies/tb3_gazebo_policy.xml \
      -n \
        /_client_node

# source overlay workspace from entrypoint
RUN sed --in-place \
      's|^source .*|source "$TB3_OVERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh && \
    cp /etc/skel/.bashrc ~/ && \
    echo 'source "$TB3_OVERLAY_WS/install/setup.bash"' >> ~/.bashrc

ENV TURTLEBOT3_MODEL burger
ENV GAZEBO_MODEL_PATH $TB3_OVERLAY_WS/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH
CMD ["byobu"]