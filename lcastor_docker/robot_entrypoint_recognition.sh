#!/bin/bash

ROBOT_NUM=$1
ETH=$2

set -e

# setup environment
source "$HOME/.bashrc"

echo " "
echo "###"
echo "### This container is part of LCASTOR!"
echo "### Report any issues to https://github.com/LCAS/LCASTOR/issues"
echo "### Entering into lcastor_recognition Docker Image"
echo " "

{

#  echo "10.8.0.102 tiago-29c" >> /etc/hosts
#  echo "10.8.0.106 tiago-89c" >> /etc/hosts
#  echo "10.8.0.105 tiago-125c" >> /etc/hosts
#  echo "192.168.1.29 tiago-29c" >> /etc/hosts
#  echo "192.168.1.89 tiago-89c" >> /etc/hosts
#  echo "192.168.1.125 tiago-125c" >> /etc/hosts
  echo "10.68.0.1 tiago-125c" >> /etc/hosts

  echo "Container is now running."
  echo " "
  if [ -z "$ROBOT_NUM" ]; then
    echo "ERROR: No argument supplied!"
    echo " "
    echo "You should run as:   ./run_robot_docker.sh ROBOT_NUM ETH"
    echo "         (example:   ./run_robot_docker.sh 125 true)"
    echo " "
    exit 1
  else
    echo "It will connect to tiago ${ROBOT_NUM} ${ETH}"
  fi 
   echo "source /home/lcastor/ros_ws/src/LCASTOR/scripts/connect_tiago.sh ${ROBOT_NUM} ${ETH}" >> /home/lcastor/.bashrc
   echo "echo \"==========================\"" >> /home/lcastor/.bashrc
   echo "echo \"WE ARE IN ARENA:\"" >> /home/lcastor/.bashrc
   echo "echo \"                 \$(rosparam get /arena)\"" >> /home/lcastor/.bashrc
   echo "echo \"To set, e.g.: rosparam set /arena arena_b\"" >> /home/lcastor/.bashrc
   echo "echo \"==========================\"" >> /home/lcastor/.bashrc



   catkin build
   source /home/lcastor/ros_ws/devel/setup.bash
       echo "function s(){  tmule -c $(rospack find robocup_human_sensing)/tmule/reco_launch.yaml -W 3 launch ; }" >> /home/lcastor/.bashrc
   echo "function t(){  tmule -c $(rospack find robocup_human_sensing)/tmule/reco_launch.yaml terminate ; }" >> /home/lcastor/.bashrc
   echo "function r(){  tmule -c $(rospack find robocup_human_sensing)/tmule/reco_launch.yaml -W 3 relaunch ; }" >> /home/lcastor/.bashrc
   mkdir /home/lcastor/ros_ws/src/LCASTOR/robocup_human_sensing/database
   chmod 777 /home/lcastor/ros_ws
   echo $(ls -la /home/lcastor/ros_ws/src/LCASTOR/robocup_human_sensing/)
   exec su lcastor 
  # tmule -c $(rospack find robocup_human_sensing)/tmule/reco_launch.yaml launch

  
   

} || {

  echo "Container failed."
  exec "$@"

}
