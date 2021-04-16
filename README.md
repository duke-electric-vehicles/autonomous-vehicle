# Autonomous-Vehicle Overview

Using ROS2 Foxy to create an autonomous vehicle. Currently working on implementing the MicroRos architecture to interface with Teensies for motor control.

# Environment Setup

## 1. Initializing this Repository
Since this repo uses the Micro Ros package (a git submodule), run these commands to clone this repo and pull the latest version of Micro Ros into this repo
```
git clone git@github.com:duke-electric-vehicles/autonomous-vehicle.git

git submodule update --init
```

## 2. Building the Docker Container
This repo includes a Dockerfile and Docker-Compose configuration that configures our custom docker image. Run the following command to build the docker image:
```
docker-compose build
```
Then, whenever you want to run the docker container, run:
```
docker-compose up -d
```
This will start a container in the background and mount `dev_ws/` to `/root/dev_ws` in the docker container. To enter the container run:
```
docker exec -it dev /bin/bash
```
You can run this command from other terminals to run multiple commands at once in the same container. When you are finished, exit the container with the ```exit``` command and run the following command to shutdown the container:
```
docker-compose down
```
## 3. Building the Workspace
First, start up a docker container and enter it. All ROS commands must be run inside that docker container, as that is where ROS is installed and accessible. Now, change directory into the dev_ws folder:
```
cd ~/dev_ws
```
ROS2 uses the `colcon` build tool. This replaces `catkin_make` from ROS1. So, in order to build the current packages in `~/dev_ws/src`, including the micro_ros_setup package, run this command from the root of the workspace, `~/dev_ws`:
```
colcon build
```
This will build our workspace and manage environment variables. In order to use commands that refer to new packages, we need to make the terminal aware of these changes. Run the following command to do this (and re-run this evertime you rebuild the workspace):
```
. install/setup.bash
```
Now your workspace is built and you should see new folders in the `~/dev_ws` directory, namely `build`, `install`, `log`, and `src`.

## 4. Building the Micro-ROS agent
Micro-ROS allows us to communicate with microcontrollers, such as Teensies, over ROS topics. Code is uploaded onto the Teensy, and the Micro-ROS agent is run on the host machine which facilitates serial communication. The following process will create the Micro-ROS agent package using the given ```micro_ros_setup``` package already present in this repository.

First, from the `~/dev_ws` directory, update the system and ROS dependencies:
```
apt update

rosdep update
```
Now install any needed dependencies with
```
rosdep install --from-path src --ignore-src -y
```
Since we just made many changes to the workspace, we need to rebuild it and source the setup file.
```
colcon build

. install/setup.bash
```
Now that we have ensured all dependencies are updated and built, we are ready to create and build the `micro_ros_agent` package. The following command will download the agent packages.
```
ros2 run micro_ros_setup create_agent_ws.sh
```
Now, build it with:
```
ros2 run micro_ros_setup build_agent.sh
```
For good measure, rebuild the workspace and source the setup file
```
colcon build

. install/setup.bash
```

**Your Docker Environment is now fully setup. The next section details how to run the agent and upload code to the Teensy**

# Establishing Communication between Host and Teensy
We will be using the [Micro ROS Arduino](https://github.com/micro-ROS/micro_ros_arduino) package to compile ROS code through the Arduino IDE.

## 1. Patching Teensyduino to work with Micro ROS Arduino
First, download the [Latest Release](https://github.com/micro-ROS/micro_ros_arduino/releases) .zip file from the Micro ROS Arduino GitHub.

Then, in the Arduino IDE, go to `Sketch -> Include library -> Add .ZIP Library...` and add the .zip file you just downloaded. In order for the IDE to be able to compile the library successfully, run the following commmands from your Arduino / Teensyduino installation folder.
```
export ARDUINO_PATH=[Your Arduino + Teensiduino path]

cd $ARDUINO_PATH/hardware/teensy/avr/

curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/foxy/extras/patching_boards/platform_teensy.txt > platform.txt
```
Now, you should be able to boot up the Arduino IDE, pick one of the examples on the Micro ROS Arduino Github, such as the [publisher](https://github.com/micro-ROS/micro_ros_arduino/blob/foxy/examples/micro-ros_publisher/micro-ros_publisher.ino), and complie / upload it successfully to a Teensy.

## 2. Running Micro ROS Agent and Recieving Published Data from the Teensy
If you haven't, upload the publisher example to the Teensy. Now, plug it in to the host system (your computer or the Odroid). In order to find the device name, run:
```
ls /dev/serial/by-id/*
```
Copy the full line corresponding to the Teensy. Now, run the Micro ROS Agent and specify the device name:
```
ros2 run micro_ros_agent micro_ros_agent serial --dev <TEENSY DEVICE NAME>
```
If the light on the Teensy is blinking rapidly, it has entered a failed error state. This is because there is a built in timer that waits for serial communication, and fails if it does not hear from the host computer after some time. In this case, just cancel the agent command with `Ctrl-C`, press the button on the Teensy to restart the program, and rerun the agent command. It should now indicate that the session has started.

To confirm the host is hearing the published data from the Teensy, open up a new terminal, keeping the agent commmand running, and list the active ROS topics:
```
ros2 topic list
```
You should see a topic titled `/micro_ros_arduino_node_publisher`. Now, look at the incoming data with this command:
```
ros2 topic echo /micro_ros_arduino_node_publisher
```
If you see incoming data with increasing integers, communication succeeded! Now start working on communicating with more advanced messages, such as Vector3 and Twist.

# Future Plan for Controlling a Motor
The host computer will publish to a motor control topic, with normalized values ranging between 1 (forwards) and -1 (backwards). Then, the Teensy will subscribe to this topic and transform the normalized value into 
