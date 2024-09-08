docker run -it \
    --mount type=bind,source="$(pwd)"/src,target=/src \
    --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    kthfsdv/ros-intro

source /ros_entrypoint.sh

sudo apt update
sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros
