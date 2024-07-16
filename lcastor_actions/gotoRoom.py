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
import copy
"""
Starts and stops the object detection node
"""

ROOM_DICT = {
    "table": [0.3732430094704967, -9.724292821, 0.0, 0.0, 0.0, -0.9803779204370413],
    "home": [0.1, 0.1, 0.0, 0.0, 0.0, -0.9803779204370413],
    "bookshelf": [1.8236, -0.9325, 0.0, 0.0, 0.0, -0.1052502],
    "sink": [1.8236, -0.9325, 0.0, 0.0, 0.0, -0.1052502 ],
    "desk": [1.8236, -0.9325, 0.0, 0.0, 0.0, -0.1052502 ],
    "bedroom": [1.8236, -0.9325, 0.0, 0.0, 0.0, -0.1052502 ],
    "sidetables": [1.8236, -0.9325, 0.0, 0.0, 0.0, -0.1052502 ],
    "pantry": [1.8236, -0.9325, 0.0, 0.0, 0.0, -0.1052502 ],
    "kitchen": [1.8236, -0.9325, 0.0, 0.0, 0.0, -0.1052502 ],
    "sofa": [1.8236, -0.9325, 0.0, 0.0, 0.0, -0.1052502 ],
    "tvstand": [1.8236, -0.9325, 0.0, 0.0, 0.0, -0.1052502 ],
    "livingroom": [1.8236, -0.9325, 0.0, 0.0, 0.0, -0.1052502 ],
}

ROS_PARAM = "/gotoRoom/status"
class gotoRoom(AbstractAction):

    def _start_action(self):
        rospy.set_param(ROS_PARAM, "")
        self.obj_dict = {"cup": "kitchen",
                         "bed": "bedroom",
                         "bagpack" : "livingroom"}
        
        # The following coordinates are based on the Robocup house arena (X, Y, Z, R, P, Y)
        self.room_dict = {"hallway_cabinet" : [2.62, 2.37, 0.0, 0.0, -0.59, 0.80],
                          "entrance" : [0.72, 1.41, 0.0 , 0.0, -0.59, 0.80],
                          "desk_p" : [-2.19, 1.67, 0.0, 0.0, -0.04, 0.99],
                          "shelf_p" : [-3.03, 1.60, 0.0, 0.0, -0.69, 0.72],
                          "coathanger" : [0.45, 2.79, 0.0, 0.0, 0.99, 0.07],
                          "exit" : [-4.49, 3.42, 0.0, 0.0, 0.99, -0.03],
                          "TV_table_p" : [1.75, 6.01, 0.0, 0.0, -0.64, 0.76],
                          "lounge_chair" : [2.27, 6.86, 0.0, 0.0, 0.68, 0.72],
                          "lamp" : [2.31, 8.44, 0.0, 0.0, 0.68, 0.73],
                          "couch" : [-0.85, 8.13, 0.0, 0.0, 0.17, 0.98],
                          "coffe_table_p" : [0.89, 6.58, 0.0, 0.0, 0.68, 0.728],
                          "trashcan" : [-1.68, 5.27, 0.0, 0.0, -0.33, 0.94],
                          "kitchen_cabinet_p" : [-4.59, 5.25, 0.0, 0.0, -0.71, 0.69],
                          "dinner_table_p" : [-3.14, 5.15, 0.0, 0.0, 0.72, 0.68],
                          "dishwasher_p" : [-3.39, 8.10, 0.0, 0.0, 0.77, 0.63],
                          "kitchen_counter_p" : [-4.69, 8.18, 0.0, 0.0, 0.78, 0.62],
                          }
        #NOTE: Assume self.params is a list of strings the first element is the name of the node to navigate to
        rospy.loginfo('Going to ' + " ".join(self.params) + ' ...')

        if "r" in self.params[0] and self.params[1] in self.room_dict:
                self.coordinates = self.room_dict[self.params[1]]
        else: 
            if self.params[0] in self.obj_dict.keys():
                self.room = self.obj_dict[self.params[0]]
                self.coordinates = self.room_dict[self.room]
            else:
                self.coordinates = None


        print(self.params)
        print(self.coordinates)

        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        if self.coordinates != None:
            if len(self.params) < 1:
                rospy.logwarn("Wrong use of action, pass the coordinates the robots needs to reach in /map frame as X_Y_Theta")
            else:
                rospy.loginfo("Connecting to /move_base AS...")
                self.client.wait_for_server()
                rospy.loginfo("Connected.")

                # Create goal
                self.goal_msg = MoveBaseGoal()
                self.goal_msg.target_pose.header.frame_id = "map"
                self.goal_msg.target_pose.header.stamp = rospy.Time.now()
                self.goal_msg.target_pose.pose.position.x = float(self.coordinates[0])
                self.goal_msg.target_pose.pose.position.y = float(self.coordinates[1])
                self.goal_msg.target_pose.pose.orientation.x = 0.0
                self.goal_msg.target_pose.pose.orientation.y = 0.0
                self.goal_msg.target_pose.pose.orientation.z = float(self.coordinates[4])
                self.goal_msg.target_pose.pose.orientation.w = float(self.coordinates[5])
                # self.goal_msg.target_pose.pose.orientation.z = math.sin(float(self.coordinates[2]) / 2)
                # self.goal_msg.target_pose.pose.orientation.w = math.cos(float(self.coordinates[2]) / 2)

                self.client.send_goal(self.goal_msg, done_cb=self._on_goTo_done)
                rospy.loginfo("Waiting for goTo result...")
                # self.client.wait_for_result()
                rospy.set_param(ROS_PARAM, "Succeded")
        else:
            rospy.set_param(ROS_PARAM, "Failed")
            self._stop_action()

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
        #TODO also make the necessary changes to make sure this returns True when the navigation has reached the goal

        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
