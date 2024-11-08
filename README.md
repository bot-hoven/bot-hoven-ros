# Bot-Hoven - ROS

## Requirements
1. Docker version 17 or higher. 
   1. Install the docker engine by following the steps [here](https://docs.docker.com/engine/install/).
   2. Manage Docker as a non-root user by following the steps [here](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user).
2. VSCode with the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension installed.

## ROS Version
This project uses **[ROS Jazzy](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html)**.

## Development Setup
1. Open the project in VSCode.
2. When prompted by the Dev Containers extension, reopen the project in the container.

## Building the Docker Image
To build the Docker image, run the following command in the root of the project:

```sh
docker build -t bot-hoven-ros -f .devcontainer/Dockerfile .