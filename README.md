# 1. Overview

This project provides an architecture and many key algorithms to control quadruped robots, including state estimation, gait generation, stance and swing leg controllers.

<!-- This project supports three control modes

- **velocity mode** allows a user to control the robot's linear and angular velocity.
- **position mode** generates user-defined gaits using gait configurations and control the robot's step position .
- **hybrid mode** uses position and torque to implement flexible locomotion.
-->


# 2. Source Code Structure

The source code includes five major directories

- **demo** has many demo examples to help users understand the software usage and the project architecture itself.
- **extern** contains the third-party dependencies to successfully compile and run the code.
- **navigation** contains the codes for SLAM and navigation.
- **quadruped** contains the core modules defining robots, state, planner, dynamics and supporting algorithms.
- **simulation** contains the configuration to run demos in simulation.

---

# 3. Installation

## 3.1 Clone the repository:

```bash
git clone https://github.com/Jellal-17/quadruped-robot.git
```
## 3.2 Build the Docker image:

```bash
docker build -t quadruped_robot .
```

## 3.3 Run the docker container:

```bash
docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --privileged \
    --gpus all \
    quadruped_robot
```
- Change the name of the container (```quadruped_robot```) as you wish.
- Modify the command if you do not want to use all GPUs by replacing ```--gpus all``` with a specific GPU index (e.g., ```--gpus '"device=0"'```).

---

# 4. Run Demos

## 4.1 Browse the demos

Browse the directories `src/demo/${demo_xxx_xxx}` to find many demo examples. The demo can either run in a Gazebo simulator or in a real environment. We support the robots provided by two companies: unitree-robotics and deep-robotics.

Our locomotion controllers support two modes:  velocity control and position control. Please check out the corresponding demos for the usage.

## 4.2 Run a demo in a simulator

First, in one terminal, source `setup.bash` to set up the development environment
```
./start_docker.sh
```

```
source $devel/setup.bash
```

Second, run the Gazebo simulator and load a robot.

```
roslaunch unitree_gazebo normal.launch rname:=a1 use_xacro:=true
```

In this command, **rname** specifies the robot you use and **use_xacro** indicates if you use URDF or XACRO description file.

Third, in a new terminal, launch a demo and run the quadruped controller node. Here, a demo helloworld lets the quadruped robot stand up.

```
./access_docker.sh
rosrun demo demo_helloworld sim
```

Here, `sim` indicates that the demo is running in simulation. For more demo examples, please check out the directory /demos. If you have a robot **YAML** configuration file such as  **XACRO** or **URDF**, you can specify the file location to initialize a **qrRobotSim** class. 
