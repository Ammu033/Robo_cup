import os
import sys
import yaml

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
from AbstractAction import AbstractAction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math
import random
import copy
"""
Starts and stops the object detection node
"""

def load_yaml_as_room_dict(yaml_path):
    """Load YAML file and convert to room dictionary format"""
    try:
        rospy.loginfo(f"Attempting to load YAML file: {yaml_path}")
        
        # Check if file exists
        if not os.path.exists(yaml_path):
            rospy.logerr(f"YAML file does not exist: {yaml_path}")
            return {}
            
        # Check if file is readable
        if not os.access(yaml_path, os.R_OK):
            rospy.logerr(f"No read permission for YAML file: {yaml_path}")
            return {}
            
        with open(yaml_path, 'r') as file:
            yaml_data = yaml.safe_load(file)
            rospy.loginfo(f"Successfully loaded YAML with {len(yaml_data)} rooms")
            
            room_dict = {}
            for room_name, pose_data in yaml_data.items():
                pos = pose_data.get('position', {})
                orient = pose_data.get('orientation', {})
                # Convert to legacy format [x, y, z, 0.0, qz, qw]
                room_dict[room_name] = [
                    pos.get('x', 0.0),
                    pos.get('y', 0.0), 
                    pos.get('z', 0.0),
                    0.0,
                    orient.get('z', 0.0),
                    orient.get('w', 1.0)
                ]
            rospy.loginfo(f"Converted {len(room_dict)} rooms to legacy format")
            return room_dict
            
    except PermissionError as e:
        rospy.logerr(f"Permission denied accessing YAML file {yaml_path}: {e}")
        return {}
    except FileNotFoundError as e:
        rospy.logerr(f"YAML file not found {yaml_path}: {e}")
        return {}
    except yaml.YAMLError as e:
        rospy.logerr(f"YAML parsing error in {yaml_path}: {e}")
        return {}
    except Exception as e:
        rospy.logerr(f"Unexpected error loading YAML {yaml_path}: {e}")
        return {}

# Load from YAML instead of hardcoded
YAML_PATH = "/home/lcastor/ros_ws/src/LCASTOR/examples/goal.yaml"
ROOM_DICT_B = load_yaml_as_room_dict(YAML_PATH)
ROOM_DICT_C = load_yaml_as_room_dict(YAML_PATH)  # Same data for both arenas

ROOM_DICT = {
    "arena_b" : ROOM_DICT_B,
    "arena_c" : ROOM_DICT_C
}

ROS_PARAM = "/gotoRoom/status"

class gotoRoom(AbstractAction):

    def _start_action(self):
        rospy.set_param(ROS_PARAM, "")
        ROOM = rospy.get_param("/arena", "arena_b")
        self.obj_dict = {
            "cup": "kitchen",
            "bed": "bedroom",
            "bagpack" : "livingroom"
        }
        
        # The following coordinates are based on the Robocup house arena (X, Y, Z, R, P, Y)
        self.room_dict = copy.deepcopy(ROOM_DICT)
        self.room_dict_b = copy.deepcopy(ROOM_DICT_B)
        self.room_dict_c = copy.deepcopy(ROOM_DICT_C)

        #NOTE: Assume self.params is a list of strings the first element is the name of the node to navigate to
        rospy.loginfo('Going to ' + " ".join(self.params) + ' ...')

        if "r" in self.params[0]:
            # Extract room name by removing "r_" prefix
            room_name = self.params[1]  # Remove "r_" prefix
            
            rospy.loginfo(f"Looking for room: {room_name}")
            if room_name in self.room_dict[ROOM]:
                self.coordinates = self.room_dict[ROOM][room_name]
                rospy.loginfo(f"Found coordinates for {room_name}")
            else:
                rospy.logerr(f"Room {room_name} not found. Available rooms: {list(self.room_dict[ROOM].keys())}")
                rospy.set_param(ROS_PARAM, "Failed")
                self._stop_action()
                return

        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        if self.coordinates != None:
            if len(self.params) < 1:
                rospy.logwarn("Wrong use of action, pass the coordinates the robots needs to reach in /map frame as X_Y_Theta")
            else:
                rospy.loginfo("Connecting to /move_base AS...")
                self.client.wait_for_server()
                rospy.loginfo("Connected.")

                # Create move base goal position
                self.goal_msg = MoveBaseGoal()
                self.goal_msg.target_pose.header.frame_id = "map"
                self.goal_msg.target_pose.header.stamp = rospy.Time.now()
                self.goal_msg.target_pose.pose.position.x = float(self.coordinates[0])
                self.goal_msg.target_pose.pose.position.y = float(self.coordinates[1])
                self.goal_msg.target_pose.pose.orientation.x = 0.0
                self.goal_msg.target_pose.pose.orientation.y = 0.0
                self.goal_msg.target_pose.pose.orientation.z = float(self.coordinates[4])
                self.goal_msg.target_pose.pose.orientation.w = float(self.coordinates[5])

                # Sending the position to the client
                self.client.send_goal(self.goal_msg, done_cb=self._on_goTo_done)
                rospy.loginfo("Waiting for goTo result...")
                rospy.set_param(ROS_PARAM, "Succeeded")

    def _on_goTo_done(self, goalState, result):
        print("goToRoom DONE", goalState, result)
        self.params.append("done")
        rospy.loginfo('Destination reached')

    def _stop_action(self):
        if hasattr(self, 'client') and self.client is not None:  # ADDED THIS CHECK
            self.client.cancel_all_goals()
            self.params.append("done")
            rospy.loginfo('STOPPED goto action')

    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached