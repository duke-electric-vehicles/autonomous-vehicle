# Autonomous-Vehicle Overview

Using ROS2 Galactic to create an autonomous vehicle. Currently working on implementing the MicroRos architecture to interface with Teensies for motor control.

# Environment Setup

## 1. Clone this Repository
Pull the latest version of this repository to your local computer:
```
git clone git@github.com:duke-electric-vehicles/autonomous-vehicle.git
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
This will start a container in the background and mount `ros_packages/` to `/opt/ros/dev_ws/src` in the docker container. To enter the container run:
```
docker exec -it dev bash
```
You can run this command from other terminals to run multiple commands at once in the same container. When you are finished, exit the container with the ```exit``` command and run the following command to shutdown the container:
```
docker-compose down
```
## 3. Building the Workspace
This step is now automatically done by the docker image. However, it is still useful to know how to build the workspace once you start adding packages and making changes. First, start up a docker container and enter it. All ROS commands must be run inside that docker container, as that is where ROS is installed and accessible. Now, change directory into the dev_ws folder:
```
cd /opt/ros/dev_ws
```
ROS2 uses the `colcon` build tool. This replaces `catkin_make` from ROS1. So, in order to build the current packages in `/opt/ros/dev_ws/src`, run this command from the root of the workspace, `/opt/ros/dev_ws`:
```
colcon build
```
This will build our workspace and manage environment variables. In order to use commands that refer to new packages, we need to make the terminal aware of these changes. Run the following command to do this (and re-run this evertime you rebuild the workspace):
```
. install/setup.bash
```
Now your workspace is built and you should see new folders in the `/opt/ros/dev_ws` directory, namely `build`, `install`, `log`, and `src`.

## 4. The Micro-ROS agent
Micro-ROS allows us to communicate with microcontrollers, such as Teensies, over ROS topics. Code is uploaded onto the Teensy, and the Micro-ROS agent is run on the host machine which facilitates serial communication. You may have noticed that when you ran `docker-compose up -d` earlier, it started two containers. One of those is for the micro ros agent.


**Your Docker Environment is now fully setup. The next section details how to run the agent and upload code to the Teensy**

## 5. Running Micro ROS Agent and Recieving Published Data from the Teensy

First, pick one of the examples on the Micro ROS Arduino Github, such as the [publisher](https://github.com/micro-ROS/micro_ros_arduino/blob/foxy/examples/micro-ros_publisher/micro-ros_publisher.ino), and complie / upload it successfully to a Teensy.

Now, plug it in to the host system (your computer or the Odroid). In order to find the device name, run:
```
ls /dev/serial/by-id/*
```
Copy the full line corresponding to the Teensy and save it for later. Now, start up the containers with
```
docker-compose up -d
```

Next, enter into the uros container with
```
docker exec -it uros bash
```
And run the following command to start the micro ros agent:
```
ros2 run micro_ros_agent micro_ros_agent serial --dev <DEVICE NAME>
```
Where \<DEVICE NAME\> is the output from the `ls /dev/serial/by-id/*` command from earlier.

If the light on the Teensy is blinking rapidly, it has entered a failed error state. This is because there is a built in timer that waits for serial communication, and fails if it does not hear from the host computer after some time. In this case, just replug the Teensy into your USB port.

To confirm the host is hearing the published data from the Teensy, open up a new terminal, enter the dev container with `docker exec -it uros bash`, and list the active ROS topics:
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
