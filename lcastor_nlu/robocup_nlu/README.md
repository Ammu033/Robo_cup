# robocup_nlu
This repository contains a ROS package that allows Tiago robots to transform user speech to text, to understand that text, and to interact accordingly (by responding or by activating a robot action).
# How to use the code:
PREREQUISITES:

The robocup_nlu package requires the following packages/dependencies in order to be used. Make sure that all packages are cloned into the directory `~/<workspace name>/src` where `<workspace name>` is your workspace name (the default name used is `catkin_ws`).

1. Install ROS Noetic following the steps shown [here](http://wiki.ros.org/noetic/Installation/Ubuntu). 
2. Clone the robocup_nlu repository into `~/<workspace name>/src/`.
3. Install conda and create a virtual environment with python 3.8.10 as indicated in [here](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html). 
4. Inside the virtual environment, run the script `~/<workspace name>/src/robocup_nlu/config/install_deps.sh` to install the dependencies and necessary ROS packages.
5. Finally, make sure to build and source the workspace.

HOW TO USE IT:
* The config files into the (`~/robocup_nlu/tmule/` directory) have several parameters that can be modified in case some features of are not required for certain tests, e.g. not running the speech to text feature. Moreover, directories of important files can be modified using these configuration parameters, e.g. the name of the workspace folder, the name of the conda environment. etc. Make sure to modify the directories according to your system.

To launch any config file into the `~/robocup_nlu/tmule/` directory, it is necessary have installed Tmule-TMux Launch Engine (it was already installed in step 4 of prerequisites), and execute the following commands in terminal:
```
roscd robocup_nlu/tmule
tmule -c <config_file_name>.yaml launch
```
To terminate the execution of a specific tmule session:
```
tmule -c <config_file_name>.yaml terminate
```
To monitor the state of every window and panel launched for the current active tmule sessions:
```
tmux a
```
To navigate through the windows in the current tmule sessions: `CTRL + B` and then `W`.
To jump to different panels within the same window: `CTRL + B` and then `UP` or `DOWN`.
To scroll up/down within a panel: `CTRL + B` and then `PG UP` or `PG DOWN`. To quit that mode: `Q`
To leave the tmule session visualization: `CTRL + B` and then `D`.
