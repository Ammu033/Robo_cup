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
   tmule -c $(rospack find robocup_human_sensing)/tmule/reco_launch.yaml launch
   exec su lcastor

} || {

  echo "Container failed."
  exec "$@"

}
