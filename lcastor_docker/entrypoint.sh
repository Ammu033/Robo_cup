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
   source /ros_ws/devel/setup.bash
   /bin/bash

} || {

  echo "Container failed."
  exec "$@"

}
