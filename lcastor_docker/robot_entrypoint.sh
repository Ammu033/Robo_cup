#!/bin/bash

ROBOT_NUM=$1
#IFACE=$2

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

  echo "10.8.0.102 tiago-29c" >> /etc/hosts
  echo "10.8.0.106 tiago-89c" >> /etc/hosts
  echo "10.8.0.105 tiago-125c" >> /etc/hosts
  su-exec lcastor "$@"

  echo "Container is now running."
  echo " "
  if [ -z "$ROBOT_NUM" ]; then
    echo "ERROR: No argument supplied!"
    echo " "
    echo "You should run as:   ./run_robot_docker.sh ROBOT_NUM"
    echo "         (example:   ./run_robot_docker.sh 125)"
    echo " "
    exit 1
  else
    echo "It will connect to tiago ${ROBOT_NUM} "
  fi 

   echo "source /home/lcastor/ros_ws/src/LCASTOR/scripts/connect_tiago.sh ${ROBOT_NUM}" >> /home/lcastor/.bashrc
   catkin build
   source /home/lcastor/ros_ws/devel/setup.bash
   /bin/bash
   

} || {

  echo "Container failed."
  exec "$@"

}
