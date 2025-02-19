#!/bin/bash

image_name=lcas.lincoln.ac.uk/lcastor/lcastor_base_ollama

xhost + local:docker

echo "Starting docker container..."
docker run --privileged --network host \
           --env="DISPLAY=$DISPLAY" \
           --env="QT_X11_NO_MITSHM=1" \
           --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
           -v $(pwd)/../:/home/lcastor/ros_ws/src/LCASTOR \
           -v /dev/dri:/dev/dri \
           -v /dev/snd:/dev/snd \
           --group-add=audio \
           --rm \
           -it ${image_name}
