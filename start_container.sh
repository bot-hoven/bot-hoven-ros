#!/bin/bash

# Variables
IMAGE_NAME="bothoven-ros"
CONTAINER_NAME="bothoven-ros-container"
USERNAME="ubuntu"
WORKSPACE_DIR="$(pwd)"
WORKSPACE_NAME="$(basename $WORKSPACE_DIR)"

# Stop and remove existing container if it exists
echo "Stopping and removing any existing container..."
docker stop $CONTAINER_NAME >/dev/null 2>&1
docker rm $CONTAINER_NAME >/dev/null 2>&1

# Build the Docker image
# echo "Building Docker image..."
# docker build -t $IMAGE_NAME -f Dockerfile .

# Run the container
echo "Starting container..."
docker run -it --name $CONTAINER_NAME \
  --privileged \
  --network=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /sys/class/gpio:/sys/class/gpio \
  -v /dev:/dev \
  --cap-add=sys_nice \
  --ulimit memlock=-1:-1 \
  --ulimit rtprio=99 \
  -v "$WORKSPACE_DIR:/$WORKSPACE_NAME" \
  -v "$HOME/.bash_history:/home/$USERNAME/.bash_history" \
  -v bothoven-bashhistory:/commandhistory \
  -v /etc/localtime:/etc/localtime:ro \
  -w "/$WORKSPACE_NAME" \
  $IMAGE_NAME \
  /bin/bash -c "for dev in /dev/i2c-1 /dev/i2c-13 /dev/i2c-14; do if [ -e \$dev ]; then sudo chown :i2c \$dev && sudo chmod g+rw \$dev; fi; done && if [ -e /dev/spidev0.0 ]; then sudo chown :spi /dev/spidev0.0 && sudo chmod g+rw /dev/spidev0.0; fi && source /opt/ros/\$ROS_DISTRO/setup.bash && bash"

echo "Container started successfully"