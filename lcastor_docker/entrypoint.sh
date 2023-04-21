#!/bin/bash

set -e
source "/opt/ros/noetic/setup.bash"

echo " "
echo "###"
echo "### This container is part of LCASTOR!"
echo "### Report any issues to https://github.com/LCAS/LCASTOR/issues"
echo "###"
echo " "

{

  echo "Container is now running."
  echo " "

  /bin/bash
  
} || {

  echo "Container failed."
  exec "$@"

}