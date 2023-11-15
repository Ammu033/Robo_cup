#!/bin/bash

set -e

# setup environment
source "$HOME/.bashrc"

# Cleanup to be "stateless" on startup, otherwise pulseaudio daemon can't start
#rm -rf /var/run/pulse /var/lib/pulse /root/.config/pulse

# Start pulseaudio as system wide daemon; for debugging it helps to start in non-daemon mode
#pulseaudio -D --verbose --exit-idle-time=-1 --system --disallow-exit

# Create a virtual audio source; fixed by adding source master and format
#echo "Creating virtual audio source: ";
#pactl load-module module-virtual-source master=auto_null.monitor format=s16le source_name=VirtualMic

# Set VirtualMic as default input source;
#echo "Setting default source: ";
#pactl set-default-source VirtualMic


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
   echo "function s(){  tmule -c $(rospack find robocup_nlu)/tmule/docker_stt_nlu.yaml -W 3 launch ; }" >> /home/lcastor/.bashrc
   echo "function t(){  tmule -c $(rospack find robocup_nlu)/tmule/docker_stt_nlu.yaml terminate ; }" >> /home/lcastor/.bashrc
   echo "function r(){  tmule -c $(rospack find robocup_nlu)/tmule/docker_stt_nlu.yaml -W 3 relaunch ; }" >> /home/lcastor/.bashrc

  #  tmule -c $(rospack find robocup_nlu)/tmule/docker_stt_nlu.yaml -W 3 launch
   exec su lcastor
   tmule -c $(rospack find robocup_nlu)/tmule/docker_stt_nlu.yaml -W 3 launch


} || {

  echo "Container failed."
  exec "$@"

}
