#!/bin/bash

image_name=lcastor_base

echo "Starting docker container..."
docker run --network host \
           -v $(dirname "$0")/../:/ros_ws/src/LCASTOR \
           --gpus all \
           -e ROS_MASTER_URI=${ROS_MASTER_URI} \
           -e ROS_IP=${ROS_IP} \
           --name "${image_name/:/-}" \
           --rm \
           -it ${image_name} \
           bash -c "echo ciao & /bin/bash"
