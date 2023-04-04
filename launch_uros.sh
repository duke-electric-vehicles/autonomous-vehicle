docker run -it --net=host -v /dev:/dev --privileged \
  microros/micro-ros-agent:foxy serial --dev \
  /dev/serial/by-id/usb-Teensyduino_USB_Serial_5886130-if00
