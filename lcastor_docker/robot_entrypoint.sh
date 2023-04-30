#!/bin/bash

ROBOT_NUM=$1
IFACE=$2

set -e

# setup environment
source "$HOME/.bashrc"

echo " "
echo "###"
echo "### This container is part of LCASTOR!"
echo "### Report any issues to https://github.com/LCAS/LCASTOR/issues"
echo "###"
echo " "

{

  echo "Container is now running."
  echo " "
  if [ -z "$ROBOT_NUM" ] || [ -z "$IFACE" ]; then
    echo "ERROR: No arguments supplied!"
    echo " "
    echo "You should run as:   ./run_robot_docker.sh ROBOT_NUM IFACE"
    echo "         (example:   ./run_robot_docker.sh 125 wlp0s20f3)"
    echo " "
    exit 1
  else
    echo "It will connect to tiago ${ROBOT_NUM} via ${IFACE} "
  fi 

   echo "source /home/lcastor/ros_ws/src/LCASTOR/scripts/connect_tiago.sh ${ROBOT_NUM} ${IFACE}" >> /home/lcastor/.bashrc
   catkin build
   source /home/lcastor/ros_ws/devel/setup.bash
   /bin/bash
   

} || {

  echo "Container failed."
  exec "$@"

}
