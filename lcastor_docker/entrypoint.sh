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

   catkin build
   source /home/lcastor/ros_ws/devel/setup.bash
   exec su lcastor

} || {

  echo "Container failed."
  exec "$@"

}
