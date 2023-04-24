#!/bin/bash

image_name=lcastor_base

xhost +

echo "Starting docker container..."
docker run --privileged --network host \
           --gpus all \
           --env="NVIDIA_DRIVER_CAPABILITIES=all" \
           --env="DISPLAY=$DISPLAY" \
           --env="QT_X11_NO_MITSHM=1" \
           --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
           -v $(pwd)/../:/ros_ws/src/LCASTOR \
           -v /dev/dri:/dev/dri \
           --name "${image_name/:/-}" \
           --rm \
           -it ${image_name}
        #    -e ROS_MASTER_URI=${ROS_MASTER_URI} \
        #    -e ROS_IP=${ROS_IP} \ 
        #    bash -c "echo ciao & /bin/bash"
