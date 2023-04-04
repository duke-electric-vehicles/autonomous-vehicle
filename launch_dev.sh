#!/bin/bash

docker run -it --net=host -e DISPLAY=${DISPLAY} -v /dev:/dev -v /$(pwd)/.bashrc-docker:/root/bashrc \
  -v /$(pwd)/ros_packages/:/opt/ros/dev_ws/src --privileged \
  dev_image:foxy ros2 run driver_ui jackson_dashboard
