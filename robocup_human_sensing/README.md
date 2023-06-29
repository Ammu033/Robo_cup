# robocup_human_sensing
This repository contains a ROS package that allows Tiago robots to extract features from people which whom it detect in order to identify them. The features extracted include Body Posture, Gestures and Biometrics (face, clothes, age, gender, etc). 
# How to use the code:
PREREQUISITES:

The robocup_human_sensing package requires the following packages/dependencies in order to be used. Make sure that all packages are cloned into the directory `~/<workspace name>/src` where `<workspace name>` is your workspace name (the default name used is `catkin_ws`).

1. Install ROS Noetic following the steps shown [here](http://wiki.ros.org/noetic/Installation/Ubuntu). 
2. Clone the [openpose](https://github.com/CMU-Perceptual-Computing-Lab/openpose.git) repository. This repository contains the Openpose framework used to extract human skeleton features based on RGB images. Make sure to download and install the OpenPose prerequisites for your particular operating system (e.g. cuda, cuDNN, OpenCV, Caffe, Python). Follow the instructions shown [here](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation/0_index.md).
3. Clone the robocup_human_sensing repository.
4. Run the script `~/<workspace name>/src/robocup_human_sensing/config/install_deps.sh` to install the rest of dependencies and necessary ROS packages.
5. Finally, make sure to build and source the workspace.

HOW TO USE IT:
* The config files into the (`~/robocup_human_sensing/tmule/` directory) have several parameters that can be modified in case some features of are not required for certain tests, e.g. not running the robot camera by using only data from Bag files. Moreover, directories of important files can be modified using these configuration parameters, e.g. the bag files directory, the workspace directory, etc. Make sure to modify the directories according to your system.
* To use the human gesture recognition feature for first time, it is necessary to uncompress the file which contains the trained model. This file is located in the `~/robocup_human_sensing/config/` directory. In the `config` folder, you will also find a global config file named `global_config.yaml` which contains important parameters, directories, and dictionaries used for the gesture and posture recognition. Make sure to modify the directories according to your system. Especially the directory where OpenPose was installed.

To launch any config file into the `~/robocup_human_sensing/tmule/` directory, it is necessary to execute the following commands in terminal:
```
roscd robocup_human_sensing/tmule
tmule -c <config_file_name>.yaml launch
```
To terminate the execution of a specific tmule session:
```
tmule -c <config_file_name>.yaml terminate
```
To monitor the state of every panel launched for the current active tmule sessions:
```
tmux a
```
To navigate through the windows in the current tmule sessions: `CTRL + B` and then `W`.
To jump to different panels within the same window: `CTRL + B` and then `UP` or `DOWN`.
To scroll up/down within a panel: `CTRL + B` and then `PG UP` or `PG DOWN`. To quit that mode: `Q`
To leave the tmule session visualization: `CTRL + B` and then `D`.
