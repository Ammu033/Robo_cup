#!/bin/bash



image_name=lcas.lincoln.ac.uk/lcastor/lcastor_base:ui

xhost + local:docker

echo "Starting docker container..."
docker run --privileged --network host \
           --gpus all \
           --env="NVIDIA_DRIVER_CAPABILITIES=all" \
           --env="DISPLAY=$DISPLAY" \
           --env="QT_X11_NO_MITSHM=1" \
           --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
           -v $(pwd)/../:/home/lcastor/ros_ws/src/LCASTOR \
           -v /dev/dri:/dev/dri \
           --rm \
           --entrypoint /home/lcastor/ros_ws/src/LCASTOR/lcastor_docker/robot_entrypoint.sh \
           -it ${image_name}  \
           $1 $2
           #--name "${image_name/:/-}" \
        #    -e ROS_MASTER_URI=${ROS_MASTER_URI} \
        #    -e ROS_IP=${ROS_IP} \ 
        #    bash -c "echo ciao & /bin/bash"
