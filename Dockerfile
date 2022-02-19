ARG FROM_IMAGE=ros:galactic
ARG OVERLAY_WS=/opt/ros/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# copy overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY ./overlay ./
RUN vcs import src < overlay.repos && \
    find src -name ".git" | xargs rm -rf || true

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder
ARG DEBIAN_FRONTEND=noninteractive

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

# # install RTI Connext DDS
# # set up environment
# ENV NDDSHOME /opt/rti.com/rti_connext_dds-6.0.1
# WORKDIR $NDDSHOME
# COPY ./rti ./
# RUN yes | ./rti_connext_dds-6.0.1-eval-x64Linux3gcc5.4.0.run && \
#     mv y/*/* ./ && rm -rf y
# # set RTI DDS environment
# ENV CONNEXTDDS_DIR $NDDSHOME
# ENV PATH "$NDDSHOME/bin":$PATH
# ENV LD_LIBRARY_PATH "$NDDSHOME/lib/x64Linux3gcc5.4.0":$LD_LIBRARY_PATH
# # set RTI openssl environment
# ENV PATH "$NDDSHOME/third_party/openssl-1.1.1d/x64Linux4gcc7.3.0/release/bin":$PATH
# ENV LD_LIBRARY_PATH "$NDDSHOME/third_party/openssl-1.1.1d/x64Linux4gcc7.3.0/release/lib":$LD_LIBRARY_PATH

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep update \
      --rosdistro $ROS_DISTRO && \
    apt-get upgrade -y && \
    rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release ccache"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $OVERLAY_MIXINS

# # install RTI Connext
# ENV RTI_NC_LICENSE_ACCEPTED yes
# RUN apt-get update && apt-get install -y \
#       ros-$ROS_DISTRO-rmw-connext-cpp \
#     && rm -rf /var/lib/apt/lists/*
# # set up environment
# ENV NDDSHOME /opt/rti.com/rti_connext_dds-5.3.1
# ENV PATH "$NDDSHOME/bin":$PATH
# ENV LD_LIBRARY_PATH "$NDDSHOME/lib/x64Linux3gcc5.4.0":$LD_LIBRARY_PATH
# # install RTI Security
# WORKDIR $NDDSHOME
# # ADD https://s3.amazonaws.com/RTI/Bundles/5.3.1/Evaluation/rti_connext_dds_secure-5.3.1-eval-x64Linux3gcc5.4.0.tar.gz ./
# # RUN tar -xvf rti_connext_dds_secure-5.3.1-eval-x64Linux3gcc5.4.0.tar.gz -C ./
# COPY ./rti ./
# RUN rtipkginstall rti_security_plugins-5.3.1-eval-x64Linux3gcc5.4.0.rtipkg && \
#     rtipkginstall openssl-1.0.2n-5.3.1-host-x64Linux.rtipkg && \
#     tar -xvf openssl-1.0.2n-target-x64Linux3gcc5.4.0.tar.gz
# ENV PATH "$NDDSHOME/openssl-1.0.2n/x64Linux3gcc5.4.0/release/bin":$PATH
# ENV LD_LIBRARY_PATH "$NDDSHOME/openssl-1.0.2n/x64Linux3gcc5.4.0/release/lib":$LD_LIBRARY_PATH
# # install RTI QoS
# ENV NDDS_QOS_PROFILES "$NDDSHOME/NDDS_QOS_PROFILES.xml"

# generate artifacts for keystore
ENV TB3_DEMO_DIR $OVERLAY_WS/..
WORKDIR $TB3_DEMO_DIR
COPY policies policies
RUN . $OVERLAY_WS/install/setup.sh && \
    ros2 security generate_artifacts -k keystore \
      -p policies/tb3_gazebo_policy.xml

# copy demo files
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
