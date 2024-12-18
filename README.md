# Bot-Hoven - ROS

## Requirements
1. Docker version 17 or higher. 
   1. Install the docker engine by following the steps [here](https://docs.docker.com/engine/install/).
   2. Manage Docker as a non-root user by following the steps [here](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user).
2. VSCode with the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension installed.
3. NVIDIA Container Toolkit. (Optional)
   1. This is only required if an NVIDIA GPU will be used within the dev container.
   2. Install the NVIDIA Container Toolkit by following the steps [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installation).

## ROS Version
This project uses **[ROS Jazzy](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html)** and **[Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted/)**.

## Development Setup
1. Open the project in VSCode.
2. When prompted by the Dev Containers extension, reopen the project in the container.
3. (Optional) If GUI applications are required to run in the container, the following command will need to run on the host machine:
```sh
xhost +local:docker
```

## Building the Docker Image
To build the Docker image, run the following command in the root of the project:

```sh
docker build -t bot-hoven-ros -f .devcontainer/Dockerfile .
```

## Building the Workspace
```sh
colcon build
source install/setup.bash
```

## Running the ROS Project
1. To start the nodes, run the `startup.sh` script:
```sh
./scripts/startup.sh
```
2. To stop the nodes, run the `shutdown.sh` script:
```sh
./scripts/shutdown.sh
```

## Checking the URDF
```sh
ros2 run xacro xacro src/description/urdf/bothoven.urdf.xacro -o <output-file>.urdf
check_urdf <output-file>.urdf
```