#!/bin/bash

docker run -it --net=host -e DISPLAY=${DISPLAY} -v /dev:/dev -v ./.bashrc-docker:/root/bashrc \
  -v ./ros_packages/:/opt/ros/dev_ws/src --privileged \
  dev_image:foxy ros2 run driver_ui dashboard
