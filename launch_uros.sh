docker run -it --net=host --restart always -v /dev:/dev --privileged \
  microros/micro-ros-agent:foxy serial --dev \
  /dev/serial/by-id/usb-Teensyduino_USB_Serial_13453950-if00
