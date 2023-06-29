#!/bin/bash

### INSTALL PREREQUISITES

# TMULE
pip install tmule
sudo apt install tmux
echo "Tmule installed"

# JOBLIB
sudo apt-get install -y python3-joblib
echo "Joblib installed"

# SKLEARN
pip3 install scipy
pip install -U scikit-learn==0.22
echo "Scikit-learn installed"

#WEB_VIDEO_SERVER
if [ -d "~/catkin_ws/src/web_video_server" ] 
then
    echo "ROS package web_video_server exists" 
else
    cd ~/catkin_ws/src/
    git clone https://github.com/RobotWebTools/web_video_server
    cd ~/catkin_ws
    catkin_make --only-pkg-with-deps web_video_server
    source devel/setup.bash
    echo "ROS package web_video_server installed"
fi

# Video_stream_opencv ros package (just to test human detection with laptop webcam)
if [ -d "~/catkin_ws/src/ video_stream_opencv" ] 
then
    echo "ROS package video_stream_opencv exists" 
else
    cd ~/catkin_ws/src/
    git clone https://github.com/ros-drivers/video_stream_opencv.git
    cd ~/catkin_ws
    catkin_make --only-pkg-with-deps video_stream_opencv
    source devel/setup.bash
    echo "ROS package video_stream_opencv installed"
fi

# Realsense ROS wrapper (just to test human detection with realsense camera)
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
echo "Realsense ROS wrapper installed"

# Darknet ROS (it may produce an error if you have GPU and CUDA installed, see README to address that error by modifying the CMakeLists.txt file)
if [ -d "~/catkin_ws/src/darknet_ros" ] 
then
    echo "ROS package darknet_ros exists" 
else
    cd ~/catkin_ws/src/
    git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
    cd ~/catkin_ws
    catkin_make -DCMAKE_BUILD_TYPE=Release --only-pkg-with-deps darknet_ros
    source devel/setup.bash
    echo "ROS package darknet_ros installed"
fi

# All the other human detection dependencies
#cd ~/catkin_ws/src/robocup_human_sensing/config/
#pip install -r human_detection_requirements.txt
#echo "Human detection dependencies installed"

