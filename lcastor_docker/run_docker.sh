#!/bin/bash

image_name=lcastor_base

echo "Starting docker container..."
docker run --network host \
           --gpus all \
           --env="NVIDIA_DRIVER_CAPABILITIES=all" \
           --env="DISPLAY" \
           --env="QT_X11_NO_MITSHM=1" \
           -v $(dirname "$0")/../:/ros_ws/src/LCASTOR \
           -v /dev/dri:/dev/dri \
           --name "${image_name/:/-}" \
           --rm \
           -it ${image_name}
        #    -e ROS_MASTER_URI=${ROS_MASTER_URI} \
        #    -e ROS_IP=${ROS_IP} \ 
        #    bash -c "echo ciao & /bin/bash"
