# Autonomous-Vehicle Overview

Using ROS2 Foxy to create an autonomous vehicle. Currently working on implementing the MicroRos architecture to interface with Teensies for motor control.

# Environment Setup

## 1. Initializing this Repository
Since this repo uses the Micro Ros package (a git submodule), run these commands to clone this repo and pull the latest version of Micro Ros into this repo
```
git submodule update --init

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
