#!/bin/bash
set -e  # Exit on error

CONTAINER_NAME="ros_melodic_dev"

echo "🚀 Starting Docker container with ROS Melodic..."

# 1. Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "❌ Docker is not installed or not in your PATH."
    exit 1
fi

# 2. Allow X11 access for GUI applications
echo "🔓 Allowing X11 access to local docker..."
xhost +local:docker || { echo "❌ Failed to set xhost permissions"; exit 1; }

# 3. Set volume paths
HOST_ROS_WS_PATH="$HOME/Documents/5LIA0_Embedded_Visual_Control/evc_group_4"
CONTAINER_ROS_WS_PATH="/ros_ws"

if [ ! -d "$HOST_ROS_WS_PATH" ]; then
    echo "❌ ROS workspace path does not exist: $HOST_ROS_WS_PATH"
    exit 1
fi

# 4. Check container state
container_exists=$(docker ps -a --filter "name=^/${CONTAINER_NAME}$" --format "{{.Names}}")

if [ "$container_exists" = "$CONTAINER_NAME" ]; then
    echo "🔁 Container '$CONTAINER_NAME' already exists."

    container_running=$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")

    if [ "$container_running" = "true" ]; then
        echo "📎 Attaching to running container '$CONTAINER_NAME'..."
        docker exec -it "$CONTAINER_NAME" bash
    else
        echo "🔄 Restarting and attaching to stopped container '$CONTAINER_NAME'..."
        docker start -ai "$CONTAINER_NAME"
    fi
else
    echo "🐳 Creating and starting new container '$CONTAINER_NAME'..."
    docker run -it \
        --name "$CONTAINER_NAME" \
        --env="DISPLAY=${DISPLAY}" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="${HOST_ROS_WS_PATH}:${CONTAINER_ROS_WS_PATH}" \
        ros:melodic bash
fi

echo "✅ Done"
