ARG FROM_IMAGE=ros:foxy
ARG OVERLAY_WS=/opt/ros/dev_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
ADD ros_packages .

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/opt && \
    find ./ -name "COLCON_IGNORE" | \
      xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder

RUN apt-get -y update
RUN apt-get -y upgrade
RUN apt-get install -y build-essential net-tools vim wget
RUN apt-get install -y nano
RUN apt-get install -y python3-pip
RUN apt-get install -y ros-foxy-rqt*

RUN pip3 install Cython
RUN pip3 install sbp
RUN pip3 install numpy --upgrade

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS/src ./src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-paths \
        src/ \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
COPY --from=cacher $OVERLAY_WS/src ./src
ARG OVERLAY_MIXINS="release"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --mixin $OVERLAY_MIXINS

RUN apt-get update
RUN apt-get install -y curl

RUN curl -fsSL https://deb.nodesource.com/setup_19.x | bash - &&\
apt-get install -y nodejs

RUN npm install -g pyright

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
ENV LANG C.UTF-8
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh


CMD ["pyright-langserver", "--stdio"]
