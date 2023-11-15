#!/bin/bash

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

  echo "Container is now running."
  echo " "

   catkin build
   source /home/lcastor/ros_ws/devel/setup.bash
      echo "function s(){  tmule -c $(rospack find robocup_human_sensing)/tmule/reco_launch.yaml -W 3 launch ; }" >> /home/lcastor/.bashrc
   echo "function t(){  tmule -c $(rospack find robocup_human_sensing)/tmule/reco_launch.yaml terminate ; }" >> /home/lcastor/.bashrc
   echo "function r(){  tmule -c $(rospack find robocup_human_sensing)/tmule/reco_launch.yaml -W 3 relaunch ; }" >> /home/lcastor/.bashrc
  #  tmule -c $(rospack find robocup_human_sensing)/tmule/reco_launch.yaml launch
   exec su lcastor
   tmule -c $(rospack find robocup_human_sensing)/tmule/reco_launch.yaml launch


} || {

  echo "Container failed."
  exec "$@"

}
