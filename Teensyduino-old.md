# Establishing Communication between Host and Teensy

## 1. Installing Teensyduino
Teensyduino is an extension to the Arduino IDE that enables you to directly download code to a Teensy. [Teensyduino Download](https://www.pjrc.com/teensy/td_download.html)

## 2. Patching Teensyduino to work with Micro ROS Arduino
We will be using the [Micro ROS Arduino](https://github.com/micro-ROS/micro_ros_arduino) package to compile ROS code through the Arduino IDE.
First, download the [Latest Release](https://github.com/micro-ROS/micro_ros_arduino/releases) .zip file from the Micro ROS Arduino GitHub.

Then, in the Arduino IDE, go to `Sketch -> Include library -> Add .ZIP Library...` and add the .zip file you just downloaded. In order for the IDE to be able to compile the library successfully, run the following commmands from your Arduino / Teensyduino installation folder.
```
export ARDUINO_PATH=[Your Arduino + Teensiduino path]

cd $ARDUINO_PATH/hardware/teensy/avr/

curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/foxy/extras/patching_boards/platform_teensy.txt > platform.txt
```
Now, you should be able to boot up the Arduino IDE, pick one of the examples on the Micro ROS Arduino Github, such as the [publisher](https://github.com/micro-ROS/micro_ros_arduino/blob/foxy/examples/micro-ros_publisher/micro-ros_publisher.ino), and complie / upload it successfully to a Teensy.

## 3. Running Micro ROS Agent and Recieving Published Data from the Teensy
If you haven't, upload the publisher example to the Teensy. Now, plug it in to the host system (your computer or the Odroid). In order to find the device name, run:
```
ls /dev/serial/by-id/*
```
Copy the full line corresponding to the Teensy and replace the command argument "TeensyName" in `docker-compose.yml` under the "uros" service. Now, run the Micro ROS Agent by running 
```
docker-compose up -d
```
If the light on the Teensy is blinking rapidly, it has entered a failed error state. This is because there is a built in timer that waits for serial communication, and fails if it does not hear from the host computer after some time. In this case, just bring down the docker containers, press the button on the Teensy to restart the program, and rerun the `docker-compose up -d` command.

To confirm the host is hearing the published data from the Teensy, open up a new terminal, enter the dev container with `docker exec -it dev bash`, and list the active ROS topics:
```
ros2 topic list
```
You should see a topic titled `/micro_ros_arduino_node_publisher`. Now, look at the incoming data with this command:
```
ros2 topic echo /micro_ros_arduino_node_publisher
```
If you see incoming data with increasing integers, communication succeeded! Now start working on communicating with more advanced messages, such as Vector3 and Twist.

# Controlling a Motor
The host computer will publish to a motor control topic, with normalized values ranging between 1 (forwards) and -1 (backwards). Then, the Teensy will subscribe to this topic and transform the normalized value into a pwm output for motor control.