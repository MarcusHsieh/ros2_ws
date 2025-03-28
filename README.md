
# Setup
> You should install ROS2 Humble first.

## Initial local setup for workspace (before getting packages)
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

## Setup `~/.bashrc`
> This should be in ~/.bashrc for all machines
> Only `ROS_DOMAIN_ID` is required for the jetson nano (the rest is set automatically from the docker image)
> To manually add (laptop): `sudo nano ~/.bashrc` 
> To check: `env | grep ROS`
```bash
# Source ROS2 Humble environment
source /opt/ros/humble/setup.bash
# Source workspace overlay
source ~/ros2_ws/install/setup.bash
# Setup ROS2 network configuration (for same network)
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=30
# Additional information
export ROS_ROOT=/opt/ros/humble
export ROS_VERSION=2
export ROS_PYPTHON_VERSION=3
export ROS_DISTRO=humble
```

# General Development
## Create custom packages
### Create C++ Package
```bash
ros2 pkg create my_cpp_pkg --dependencies rclcpp std_msgs --build-type ament_cmake --license MIT
```

### Create Python Package
```bash
ros2 pkg create my_python_pkg --dependencies rclpy std_msgs --build-type ament_python --license MIT
```

## You should have a `.vscode/c_cpp_properties.json` at root of workspace
> This removes any issues with including ROS headers in code
```json
{
    "configurations": [
      {
        "name": "Linux",
        "includePath": [
          "${workspaceFolder}/**",
          "/opt/ros/humble/include/**"
        ],
        "defines": [],
        "compilerPath": "/usr/bin/g++",
        "cStandard": "c11",
        "cppStandard": "c++14",
        "intelliSenseMode": "linux-gcc-x64"
      }
    ],
    "version": 4
  }
```

# Jetson Nano Instructions
## Pull ros-core (ROS2 Humble)
```bash
docker pull dustynv/ros:humble-ros-core-l4t-r32.7.1
```

## Run on ROS container with bind mount
> https://hub.docker.com/r/dustynv/ros/tags?name=humble

Run command:
```bash
./run/jetson.sh
```

## Build ROS workspace
> Root of workspace
```bash
cd /workspace
```
> Cleans old build artifacts (use if colcon build fails)
```bash
rm -rf build install log
```
> For final build
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
> For developing
> `--symlink-install`: basically compiles only new code 
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
```bash
source install/setup.bash
```

# Test if running (laptop + Jetson Nano)

## Nodes + Topics
> - [Topic = Bulletin board] [Node = Worker]
>   - **Analogy**: Where workers can see or post information
>   - **Technical**: Where nodes can subscribe to or publish information

> - **Node**: Process that performs a computation, controls hardware, or manages a specific task
>   - Control a motor (`/motor_controller`)
>   - Read a sensor (`/camera_driver`)
>   - Plan a path (`/path_planner`)
>   - Provide UI 

> - **Topic**: A bus or channel where nodes exchange messages, pathway for communication
>   - Facilitates *asychronous*, *many to many* communication
>   - Publishing nodes and subscribing nodes do NOT know each other, they only agree on the **topic name** and **type of data** being sent
>   - Topics have **unique names** and a **defined message type**
>     - `/camera/image_raw` topic carries raw image data (Message type: `sensor_msgs/Image`)
>     - `/cmd_vel` topic carries velocity commands (Message type: `geometry_msgs/Twist`)
>     - `/scan` topic carries laser scan data (Message type: `sensor_msgs/LaserScan`)

```bash
ros2 node list
```
![alt text](public/node_list.png)
```bash
ros2 topic list
```
![alt text](public/topic_list.png)
## Test with listener and talker nodes
### C++
```bash
ros2 run my_cpp_pkg talker_node
```
```bash
ros2 run my_cpp_pkg listener_node
```

### Python
```bash
ros2 run my_python_pkg talker
```
```bash
ros2 run my_python_pkg listener
```

### When running a talker
#### Manually check data throughput
```bash
ros2 topic echo chatter
```

![Alt text](public/folder_structure.png)