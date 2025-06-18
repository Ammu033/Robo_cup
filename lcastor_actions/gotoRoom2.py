import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
import yaml
from AbstractAction import AbstractAction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

"""
Navigate to rooms using poses saved in YAML file
"""

ROS_PARAM = "/gotoRoom/status"
YAML_FILE_PATH = '/home/lcastor/ros_ws/src/LCASTOR/examples/goal.yaml'

class gotoRoom(AbstractAction):

    def _start_action(self):
        rospy.set_param(ROS_PARAM, "")
        
        # Get room name from parameters
        if len(self.params) < 1:
            rospy.logerr("No room name provided")
            rospy.set_param(ROS_PARAM, "Failed")
            self._stop_action()
            return
            
        room_name = self.params[0]
        rospy.loginfo(f'Going to {room_name}...')
        
        # Load pose from YAML file
        pose_data = self._load_pose_from_yaml(room_name)
        
        if pose_data is None:
            rospy.logerr(f"Room '{room_name}' not found in YAML file")
            rospy.set_param(ROS_PARAM, "Failed")
            self._stop_action()
            return
        
        # Setup move_base client
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        
        rospy.loginfo("Connecting to /move_base AS...")
        self.client.wait_for_server()
        rospy.loginfo("Connected.")
        
        # Create and send goal
        self.goal_msg = MoveBaseGoal()
        self.goal_msg.target_pose.header.frame_id = "map"
        self.goal_msg.target_pose.header.stamp = rospy.Time.now()
        
        # Set position from YAML
        self.goal_msg.target_pose.pose.position.x = pose_data['position']['x']
        self.goal_msg.target_pose.pose.position.y = pose_data['position']['y']
        self.goal_msg.target_pose.pose.position.z = pose_data['position']['z']
        
        # Set orientation from YAML
        self.goal_msg.target_pose.pose.orientation.x = pose_data['orientation']['x']
        self.goal_msg.target_pose.pose.orientation.y = pose_data['orientation']['y']
        self.goal_msg.target_pose.pose.orientation.z = pose_data['orientation']['z']
        self.goal_msg.target_pose.pose.orientation.w = pose_data['orientation']['w']
        
        self.client.send_goal(self.goal_msg, done_cb=self._on_goTo_done)
        rospy.loginfo("Waiting for goTo result...")
        rospy.set_param(ROS_PARAM, "Succeeded")

    def _load_pose_from_yaml(self, room_name):
        """Load pose from YAML file"""
        try:
            with open(YAML_FILE_PATH, 'r') as file:
                data = yaml.safe_load(file)
                if room_name not in data:
                    rospy.logerr(f"Room '{room_name}' not found in YAML file.")
                    return None
                return data[room_name]
        except Exception as e:
            rospy.logerr(f"Error loading YAML: {e}")
            return None

    def _on_goTo_done(self, goalState, result):
        print("goToRoom DONE", goalState, result)
        self.params.append("done")
        rospy.loginfo('Destination reached')

    def _stop_action(self):
        if hasattr(self, 'client'):
            self.client.cancel_all_goals()
        self.params.append("done")
        rospy.loginfo('STOPPED goto action')

    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached