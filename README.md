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

## Production and Development Environments

The `.devcontainer` directory contains separate folders for production and development environments. Each folder has its own `devcontainer.json` and `dockerfile` files.

### Selecting the Environment

To select the relevant environment, choose the appropriate `devcontainer.json` file when running the container:

- For development, use the `devcontainer.json` file located in the `.devcontainer/dev` directory.
- For production, use the `devcontainer.json` file located in the `.devcontainer/prod` directory.

### Adding the Pi user to the GPIO group

This step is only required when setting up a new Pi to run the production image for the first time.

1. Add the `pi` user to the `gpio` group:
   ```sh
   sudo usermod -aG gpio pi
   ```
2. Reboot the Raspberry Pi to apply the changes:
   ```sh
   sudo reboot
   ```

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
## How To
### Check the URDF
```sh
ros2 run xacro xacro src/description/urdf/bothoven.urdf.xacro -o <output-file>.urdf
check_urdf <output-file>.urdf
```

### Simulate in RViz

1. Build the workspace with symlink install:
   ```sh
   colcon build --symlink-install
   ```
   > **Note:** The `--symlink-install` option in the `colcon build` command creates symbolic links for the built packages instead of copying them, then we don’t need to rebuild every time we update the URDF. This allows for faster builds and easier debugging, as changes in the source files are immediately reflected in the built files. However, we need to rebuild whenever we add a new file.
   
2. Source the setup script:
   ```sh
   source install/setup.bash
   ```
3. In Terminal A, run the robot state publisher with the URDF file:
   ```sh
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro src/description/urdf/bothoven.urdf.xacro)"
   ```
4. In Terminal B, run the joint state publisher GUI:
   ```sh
   ros2 run joint_state_publisher_gui joint_state_publisher_gui
   ```
5. In Terminal C, start RViz:
   ```sh
   rviz2
   ```
6. Import the RViz configuration from the specified file:
   1. In RViz, go to `File` -> `Open Config`
   2. Choose the file `src/description/resource/rviz2_config.rviz`

### Launch `robot_state_publisher` with sim time

Launch `robot_state_publisher` with sim time using the following command:

```sh
ros2 launch description rsp.launch.py use_sim_time:=true
```

### Simulate in Gazebo

Run the command below:

```sh
export GZ_SIM_RESOURCE_PATH="/bot-hoven-ros"
ros2 launch description gazebo.launch.py
```

To run a script against the simulated robot (e.g. `examples/example_jtc.cpp`):

```
ros2 run hardware example_jtc
```