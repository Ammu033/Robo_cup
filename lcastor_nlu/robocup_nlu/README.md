# robocup_nlu
This repository contains a ROS package that allows Tiago robots to transform user speech to text (using Whisper), to understand that text (using Rasa), and to interact accordingly (by genarating a voice response or by activating a robot action).

# How to install it:
1. Install ROS Noetic following the steps shown [here](http://wiki.ros.org/noetic/Installation/Ubuntu). 
2. Create a ROS workspace, make sure that all packages are cloned into the directory `~/<workspace name>/src` where `<workspace name>` is your workspace name (the default name used is `catkin_ws`).
3. Clone the [LCASTOR](https://github.com/LCAS/LCASTOR.git) repository into `~/<workspace name>/src/`.
4. Install conda and create a virtual environment with python 3.8.10. To create a virtual environment with conda you can follow the steps indicated in [here](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html). 
5. Inside the virtual environment, go to the directory `~/<workspace name>/src/LCASTOR/lcastor_nlu/robocup_nlu/config/` and run:
```
./install_deps.sh
```
this will install all the necessary dependencies.
6. Finally, make sure to build and source the workspace.

# How to use it with Docker:

1. Install Docker following the steps shown [here](https://docs.docker.com/get-docker/).
2. Login to the LCAS registry: `docker login lcas.lincoln.ac.uk -u lcas -p lincoln`
3. Pull the docker image: `docker pull lcas.lincoln.ac.uk/lcastor/lcastor_base_nlu` 
4. Go to the directory `~/<workspace name>/src/LCASTOR/lcastor_docker`. There is a docker launch script setup to automatically connect to the robot ROS master every bash session inside the container. Simply run:
```
./run_robot_docker_nlu.sh ROBOT_NUM
```
where `ROBOT_NUM` is the robot number (29, 89 or 125). Make sure that you are connected to `TIAGO_WIFI` network (password `TIAGo29c`). For more information about using Docker and connecting to Tiago, please refer to the [Wiki](https://github.com/LCAS/LCASTOR/wiki/Connecting-to-TIAGo) page.

# How it works:

* The system consist into two parts: Speech To Text (STT) and Natural Language Understanding (NLU) that communicates between them and the Tiago planner through ROS topics.
* The STT part publishes `/user_speech` (String) and requires to subscribe to `/planner_intention` (String) published by the Tiago planner and `/rasa_confirmation` (Bool) publised by the NLU part. The STT is enable only when a message is published in `/planner_intention`. The STT is disable when a message is published in `/rasa_confirmation`.
* The NLU part publishes `/rasa_confirmation` (Bool), `/person_affirm_deny` (Bool), `/guest_name` (String), `/guest_drink` (String). The NLU part requires to subcribe to `/planner_intention` (String) published by the Tiago planner and `/user_speech` (String) publised by STT part. In the same way that the STT, the NLU is enable only when a message is published in `/planner_intention`, and it is disable after it publish a message in `/rasa_confirmation`.
* Basically, STT and NLU are activated only when a string message is published in `/planner_intention`. The string messages that are currently accepted by the RASA model include: `confirmation` that is used for general purposes, `user_confirm_arrived` `user_thanks` that are used for carry my luggage task, and `guest_name` `guest_drink` `affirm_deny` that are used for receptionist task.
* Once RASA have received one of the 6 options mentioned above, it will wait till the user say something, and based on the user answer, it will determine if that matches or not the intention that the planner was asking for. If the intentions matches, then RASA will publish `/rasa_confirmation` as True, and in some cases it will complement that with either  publishing `guest_name` `guest_drink` `affirm_deny`. On the other hand, if the user say something the planner was not asking for, or if Whisper fail to transcript correctly the user speech, or if RASA fail to extract information from the user speech (due to limited data used to train the model), then it will publish `/rasa_confirmation` as False.

# How to use it:

* There are two Tmule sessions that have to be launched to run both the STT and NLU.
* The first session is launched by using the file `~/robocup_nlu/tmule/conda_stt_nlu.yaml`. This config file only run the STT part (using Whisper) that publish a ROS topic with the user speech. This part can only be used outside the Docker container (due to problems getting access to microphone and GPU). 
* The second session is launched by using the file `~/robocup_nlu/tmule/docker_stt_nlu.yaml`. This config file contains the NLU part (using RASA) and the necessary scripts to communicate Whisper, RASA and the Tiago planner using ROS topics. This part is run inside the container.
* Make sure that the values of the parameters `CATKIN_WORKSPACE` `ROS_MASTER` `ROS_IP` inside the tmule config files are correct.

NOTES:

The config files into the (`~/robocup_nlu/tmule/` directory) have several parameters that can be modified in case some features of are not required for certain tests, e.g. not running the STT feature `USE_WHISPER`. Moreover, directories of important files can be modified using these configuration parameters, e.g. the name of the workspace folder `CATKIN_WORKSPACE`, the name of the conda environment `CONDA_VIRT_ENV`, and ROSMASTER IP configuration parameters `ROS_MASTER` `ROS_IP`. Make sure to modify these parameters/directories according to your system and/or the robot you are connected to.

To launch any config file into the `~/robocup_nlu/tmule/` directory, you need to execute the following commands in a terminal:
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
To navigate through the windows in the current tmule session: `CTRL + B` and then `W`.
To jump to different panels within the same window: `CTRL + B` and then `UP` or `DOWN`.
To scroll up/down within a panel: `CTRL + B` and then `PG UP` or `PG DOWN`. To quit that mode: `Q`
To leave the tmule session visualization: `CTRL + B` and then `D`.

# How to modify the Rasa model:
You will need to improve the performance of RASA on infering intentions but specially on extracting information from the user speech (guest names and favorite drinks). To do so, you need to increase the data that was used to train the model. For more information about RASA, please refer to the official [page](https://rasa.com/docs/rasa/nlu-training-data).
1. To include more data, you need to go the directory `~/<workspace name>/src/LCASTOR/lcastor_nlu/robocup_nlu/rasa/data` and open the config file `nlu.yml`.
2. Inside `nlu.yml` you will need to increase the list of examples for `person_drink` and `person_name` intentions. 
3. Once you have finished with the changes in `nlu.yml`. Then you have to retrain the RASA model. To do so, make sure you are inside the container or inside the conda environment, and then execute the following commands:
```
roscd robocup_nlu/rasa
rasa train
```
4. Once the trainning have been completed, a new model will be stored inside the directory `~/<workspace name>/src/LCASTOR/lcastor_nlu/robocup_nlu/rasa/models`. Next time you launch the tmule file `~/robocup_nlu/tmule/docker_stt_nlu.yaml`, this new model will be automatically chosen over the older one.

