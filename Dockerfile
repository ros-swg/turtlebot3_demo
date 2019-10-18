FROM osrf/ros:dashing-desktop
# FROM osrf/ros2:nightly

# patch rosdep for ament
RUN rosdep update && \
    apt-get purge python3-rosdep -y && \
    pip3 install git+https://github.com/ruffsl/rosdep.git@ament

# install helpful developer tools
RUN apt-get update && apt-get install -y \
      bash-completion \
      byobu \
      fish \
      glances \
      nano \
      python3-argcomplete \
      tree \
      vim \
    && cd /usr/bin && curl https://getmic.ro | bash \
    && rm -rf /var/lib/apt/lists/*

# install turtlebot external packages
RUN apt-get update && apt-get install -y \
      ros-$ROS_DISTRO-turtlebot3-cartographer \
      ros-$ROS_DISTRO-turtlebot3-navigation2 \
      ros-$ROS_DISTRO-turtlebot3-simulations \
      ros-$ROS_DISTRO-turtlebot3-teleop \
    && rm -rf /var/lib/apt/lists/*

# clone overlay package repos
ENV TB3_OVERLAY_WS /opt/tb3_overlay_ws
RUN mkdir -p $TB3_OVERLAY_WS/src
WORKDIR $TB3_OVERLAY_WS
COPY .docker/overlay.repos ./
RUN vcs import src < overlay.repos
# Install extra sources from this repo
COPY example_nodes/ src/example_nodes
# RUN vcs import src < src/ros-planning/navigation2/tools/ros2_dependencies.repos

# install overlay package dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
    && rosdep install -y \
      --from-paths src \
      --ignore-src \
      --skip-keys " \
            ament_mypy \
            libopensplice69 \
            rti-connext-dds-5.3.1 \
        " \
    && rm -rf /var/lib/apt/lists/*

# build overlay package source
# RUN touch $TB3_OVERLAY_WS/src/turtlebot3/turtlebot3_node/COLCON_IGNORE
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install

# generate artifacts for keystore
ENV TB3_DEMO_DIR $TB3_OVERLAY_WS/..
WORKDIR $TB3_DEMO_DIR
COPY policies policies
RUN . $TB3_OVERLAY_WS/install/setup.sh && \
    ros2 security generate_artifacts -k keystore \
      -p policies/tb3_gazebo_policy.xml

# install tools for footprinting and pentesting
# RUN pip3 install aztarna
WORKDIR /tmp
RUN apt-get -qq update && apt-get -y install \
      libgmp3-dev gengetopt \
      libpcap-dev flex byacc \
      libjson-c-dev unzip \
      libunistring-dev wget \
      libxml2-dev libxslt1-dev \
      libffi-dev libssl-dev && \
      rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/aliasrobotics/aztarna && \
      cd aztarna && python3 setup.py install
# install wireshark
RUN apt-get -qq update && DEBIAN_FRONTEND=noninteractive apt-get -y install tshark

# copy demo files
COPY maps maps
COPY configs configs
COPY .gazebo /root/.gazebo

# source overlay workspace from entrypoint
RUN sed --in-place \
      's|^source .*|source "$TB3_OVERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh && \
    cp /etc/skel/.bashrc ~/ && \
    echo 'source "$TB3_OVERLAY_WS/install/setup.bash"' >> ~/.bashrc

ENV TURTLEBOT3_MODEL='burger' \
    GAZEBO_MODEL_PATH=/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH
