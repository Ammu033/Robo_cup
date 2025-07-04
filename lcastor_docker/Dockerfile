FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04

# SHELL ["/bin/bash", "-c"] 

# Config
ENV TZ=Europe/London
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
ENV ROS_DISTRO noetic

# Minimal setup
RUN apt-get update && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales

# Install ROS
RUN rm /etc/apt/sources.list.d/* && \
    DEBIAN_FRONTEND=noninteractive apt update --no-install-recommends && \
    DEBIAN_FRONTEND=noninteractive apt install -y software-properties-common lsb-release curl wget apt-transport-https git tmux terminator nano htop net-tools iputils-ping sshpass iproute2 --no-install-recommends
RUN rm -rf /var/lib/apt/lists/* && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN DEBIAN_FRONTEND=noninteractive apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y ros-${ROS_DISTRO}-catkin ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-vision-msgs ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-rosbridge-server python3-catkin-tools python3-venv python3-pip python3-rosdep python3-rosinstall python-is-python3 --no-install-recommends && \
    rosdep init && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install tmux and TMuLE
RUN DEBIAN_FRONTEND=noninteractive pip install tmule

# Install Tensorflow
RUN pip3 install tensorflow tensorflow-hub

# create workspace
# RUN mkdir -p /ros_ws/src
# RUN cd /ros_ws; . /opt/ros/${ROS_DISTRO}/setup.sh


# RUN cd /ros_ws; rosdep install --from-paths src --ignore-src -r -y
# RUN catkin build

# Clone TIAGo dependencies
# RUN mkdir /home/lcastor
RUN cd /
RUN wget https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/${ROS_DISTRO}-devel/tiago_public-${ROS_DISTRO}.rosinstall
RUN rosinstall /ros_ws/src /opt/ros/${ROS_DISTRO} tiago_public-${ROS_DISTRO}.rosinstall
RUN apt update; apt upgrade -y

# clone the needed moveit packages tags
RUN cd /ros_ws/src; git clone https://github.com/ros-planning/moveit_msgs.git
RUN cd /ros_ws/src/moveit_msgs; git checkout tags/0.11.2
RUN cd /ros_ws/src; git clone https://github.com/ros-planning/moveit.git
RUN cd /ros_ws/src/moveit; git checkout tags/1.1.9

# install deps
RUN DEBIAN_FRONTEND=noninteractive rosdep install -y --from-paths /ros_ws/src --ignore-src --rosdistro ${ROS_DISTRO} --skip-keys "urdf_test omni_drive_controller orocos_kdl pal_filters libgazebo9-dev pal_usb_utils speed_limit_node camera_calibration_files pal_moveit_plugins pal_startup_msgs pal_local_joint_control pal_pcl_points_throttle_and_filter current_limit_controller hokuyo_node dynamixel_cpp pal_moveit_capabilities pal_pcl dynamic_footprint gravity_compensation_controller pal-orbbec-openni2 pal_loc_measure pal_map_manager ydlidar_ros_driver"
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-${ROS_DISTRO}-velodyne-* ros-${ROS_DISTRO}-apriltag-ros ros-${ROS_DISTRO}-rtabmap-* 

# clone and install PetriNetPlans
RUN cd /
RUN git clone -b noetic_devel https://github.com/francescodelduchetto/PetriNetPlans.git
RUN DEBIAN_FRONTEND=noninteractive apt install -y flex libfl-dev libfl2
RUN cd /PetriNetPlans/PNP && \
    mkdir build && cd build && cmake .. && \
    make install

# setting up user
ARG UNAME=lcastor
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $UNAME
RUN useradd -m -u $UID -g $GID -o -s /bin/bash $UNAME
RUN echo 'lcastor:lcastor' | chpasswd
RUN adduser $UNAME sudo
WORKDIR /home/lcastor/ros_ws
RUN cp -r /ros_ws /home/lcastor
RUN chown -R $UNAME:$UNAME /home/lcastor
RUN chmod 755 /home/lcastor
RUN echo "source /home/lcastor/ros_ws/devel/setup.bash" >> /home/lcastor/.bashrc
USER $UNAME

RUN cd /home/lcastor/ros_ws/src && \
    ln -s /PetriNetPlans/PNPros/ROS_bridge/pnp_ros . && \
    ln -s /PetriNetPlans/PNPros/ROS_bridge/pnp_msgs .
RUN echo "export PNP_HOME=/home/lcastor/ros_ws/src/pnp_ros/" >> /home/lcastor/.bashrc

# build ros ws
RUN cd /home/lcastor/ros_ws; . /opt/ros/${ROS_DISTRO}/setup.sh; catkin build
USER root

# Fix pip not working properly
RUN wget https://files.pythonhosted.org/packages/00/3f/ea5cfb789dddb327e6d2cf9377c36d9d8607af85530af0e7001165587ae7/pyOpenSSL-22.1.0-py3-none-any.whl
RUN python3 -m easy_install pyOpenSSL-22.1.0-py3-none-any.whl
#
# Create entrypoint for image
# COPY entrypoint.sh .
ENTRYPOINT ["/bin/bash", "/home/lcastor/ros_ws/src/LCASTOR/lcastor_docker/entrypoint.sh"]
