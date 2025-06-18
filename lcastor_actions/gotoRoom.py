import os
import sys

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

ROOM_DICT_B = { 
    "hallwaycabinet" : [-2.93, -4.47, 0.0, 0.0, 0.99, -0.08],
    "hallway" : [-1.41, -2.64, 0.0, 0.0, 0.95, -0.30],
    "entrance" : [-1.75, -0.25, 0.0, 0.0, -0.10, -0.99],
    "desk" : [-2.79, -0.46, 0.0, 0.0, -0.78, 0.61],
    "office" : [-0.96, -1.21, 0.0, 0.0, 0.85, 0.52],
    "studio" : [-0.96, -1.21, 0.0, 0.0, 0.85, 0.52],
    "shelf" : [-3.03, 1.55, 0.0, 0.0, 0.99, 0.03],
    "coathanger" : [-1.73, -2.89, 0.0, 0.0, 0.89, 0.44],
    "exit" : [-0.99, 1.61, 0.0, 0.0, 0.67, 0.73],
    "TVtable" : [1.01, -4.53, 0.0, 0.0, 0.99, -0.05],
    "loungechair" : [1.64, -4.85, 0.0, 0.0, -0.09, 0.99],
    "lamp" : [3.26, -5.12, 0.0, 0.0, -0.12, 0.99],
    "couch" : [3.6, -2.60, 0.0, 0.0, -0.35, 0.93],
    "coffetable" : [2.45, -3.20, 0.0, 0.0, -0.55, 0.83],
    "lounge" : [2.72, -1.96, 0.0, 0.0, -0.67, 0.73],
    "livingroom" : [2.72, -1.96, 0.0, 0.0, -0.67, 0.73],
    "trashcan" : [0.58, -1.16, 0.0, 0.0, 0.98, -0.17],
    "kitchen" : [3.34, -1.76, 0.0, 0.0, 0.84, 0.54],
    "kitchencabinet" : [0.62, 2.29, 0.0, 0.0, 0.99, 0.03],
    "dinnertable" : [1.44, 1.28, 0.0, 0.0, -0.02, 0.99],
    "dishwasher" : [3.67, 0.73, 0.0, 0.0, 0.04, 0.99],
    "kitchencounter" : [3.80, 1.98, 0.0, 0.0, 0.-0.0, 0.99],
    "inspectionpoint" : [0.19, -2.69, 0.0, 0.0, -0.48, 0.87],
    "findTrashEntrance" : [-0.61, 5.89, 0.0, 0.0, 0.05, 0.99],
    "findTrashOffice" : [0.30, 4.63, 0.0 , 0.0, 0.91, -0.41],
    "findTrashKitchen1" : [-3.18, 7.15, 0.0, 0.0, 0.97, -0.24],
    "findTrashKitchen2" : [-5.74, 10.44, 0.0, 0.0, -0.82, 0.57],
    "findTrashLivingRoom" : [-5.74, 10.40, 0.0, 0.0, -0.07, 0.99],
}

ROOM_DICT_C = { 
    "hallwaycabinet" : [2.62, 2.37, 0.0, 0.0, -0.59, 0.80],
    "hallway" : [0.05, 5.02, 0.0, 0.0, 0.74, 0.66],
    "entrance" : [-1.75, -0.25, 0.0, 0.0, -0.10, -0.99],
    "desk" : [-2.19, 1.67, 0.0, 0.0, -0.04, 0.99],
    "office" : [-2.42, 3.613, 0.0, 0.0, -0.806, 0.592],
    "studio" : [-2.42, 3.613, 0.0, 0.0, -0.806, 0.592],
    "shelf" : [-3.03, 1.60, 0.0, 0.0, -0.69, 0.72],
    "coathanger" : [0.45, 2.79, 0.0, 0.0, 0.99, 0.07],
    "exit" : [-4.49, 3.42, 0.0, 0.0, 0.99, -0.03],
    "TVtable" : [1.75, 6.01, 0.0, 0.0, -0.64, 0.76],
    "loungechair" : [2.27, 6.86, 0.0, 0.0, 0.68, 0.72],
    "lamp" : [2.31, 8.44, 0.0, 0.0, 0.68, 0.73],
    "couch" : [-0.85, 8.13, 0.0, 0.0, 0.17, 0.98],
    "coffetable" : [0.89, 6.58, 0.0, 0.0, 0.68, 0.728],
    "livingroom" : [0.89, 6.58, 0.0, 0.0, 0.68, 0.728],
    "lounge" : [0.89, 6.58, 0.0, 0.0, 0.68, 0.728],
    "trashcan" : [-1.68, 5.27, 0.0, 0.0, -0.33, 0.94],
    "kitchencabinet" : [-4.59, 5.25, 0.0, 0.0, -0.71, 0.69],
    "kitchen" : [-1.142, 7.697, 0.0, 0.0, -0.974, 0.225],
    "dinnertable" : [-3.14, 5.15, 0.0, 0.0, 0.72, 0.68],
    "dishwasher" : [-3.39, 8.10, 0.0, 0.0, 0.77, 0.63],
    "kitchencounter" : [-4.69, 8.18, 0.0, 0.0, 0.78, 0.62],
    "inspectionpoint" : [0.05, 5.02, 0.0, 0.0, 0.74, 0.66],
    "findTrashEntrance" : [0.01, 3.74, 0.0, 0.0, -0.25, 0.97],
    "findTrashOffice" : [-0.95, 3.69, 0.0 , 0.0, 0.98, -0.21],
    "findTrashKitchen1" : [-1.64, 5.28, 0.0, 0.0, 0.99, -0.02],
    "findTrashKitchen2" : [-0.99, 8.89, 0.0, 0.0, 0.99, -0.11],
    "findTrashLivingRoom" : [-0.99, 8.93, 0.0, 0.0, -0.42, 0.91],                  
}

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
            if self.params[1] in self.room_dict[ROOM]:
                self.coordinates = self.room_dict[ROOM][self.params[1]]
            else:
                rospy.set_param(ROS_PARAM, "Failed")
                self._stop_action()

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
                rospy.set_param(ROS_PARAM, "Succeded")

    def _on_goTo_done(self, goalState, result):
        print("goToRoom DONE", goalState, result)
        self.params.append("done")
        rospy.loginfo('Destination reached')

    def _stop_action(self):
        self.client.cancel_all_goals()
        self.params.append("done")
        rospy.loginfo('STOPPED goto action')

    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
