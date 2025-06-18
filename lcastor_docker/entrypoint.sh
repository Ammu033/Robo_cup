#!/bin/bash

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
   echo "function s(){  tmule -c ~/ros_ws/src/LCASTOR/lcastor_bringup/tmule/lcastor_simulated.yaml -W 3 launch ; }" >> /home/lcastor/.bashrc
   echo "function t(){  tmule -c ~/ros_ws/src/LCASTOR/lcastor_bringup/tmule/lcastor_simulated.yaml terminate ; }" >> /home/lcastor/.bashrc
   echo "function r(){  tmule -c ~/ros_ws/src/LCASTOR/lcastor_bringup/tmule/lcastor_simulated.yaml -W 3 relaunch ; }" >> /home/lcastor/.bashrc
   echo "echo \"==========================\"" >> /home/lcastor/.bashrc
   echo "echo \"WE ARE IN ARENA:\"" >> /home/lcastor/.bashrc
   echo "echo \"                 \$(rosparam get /arena)\"" >> /home/lcastor/.bashrc
   echo "echo \"To set, e.g.: rosparam set /arena arena_b\"" >> /home/lcastor/.bashrc
   echo "echo \"==========================\"" >> /home/lcastor/.bashrc

   catkin build
   source /home/lcastor/ros_ws/devel/setup.bash
   exec su lcastor
   tmule -c $(rospack find lcastor_bringup)/tmule/lcastor_simulated.yaml -W 3 launch


} || {

  echo "Container failed."
  exec "$@"

}