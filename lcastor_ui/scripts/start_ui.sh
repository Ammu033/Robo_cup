#!/bin/bash

cd $1/www

echo "starting http.server"
echo "{\"ROBOT_IP\": \"${ROS_IP}\"}" 
echo "{\"ROBOT_IP\": \"${ROS_IP}\"}" > $(pwd)/../www/env.json
python3 -m http.server 8888

#chrome --kiosk http://localhost:8000/index.html  # not easily possible to minimize chrome without having to close it 
