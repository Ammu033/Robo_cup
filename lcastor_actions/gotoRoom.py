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

"""
Starts and stops the object detection node
"""
class gotoRoom(AbstractAction):

    def _start_action(self):

        self.obj_dict = {"cup": "kitchen",
                         "bed": "bedroom",
                         "bagpack" : "livingroom"}
        
        # The following coordinates are based on the Robocup house arena
        self.room_dict = {"kitchen" : [6.83, 0.211, 0.0, 0.0, 0.99, -0.02],
                          "bedroom" : [6.91, -3.56, 0.0, 0.0, 0.45, 0.89],
                          "livingroom" : [0.78, 1.87, 0.0, 0.0, 0.45, 0.89],
                          "diningroom" : [5.19, 2.08, 0.0, 0.0, 0.02, 0.99],
                          "sink" : [4.32, 1.07, 0.0, 0.0, 0.93, -0.36],
                          "table" : [5.19, 2.08, 0.0, 0.0, 0.02, 0.99],
                          "cabinet" : [5.00, 2.93, 0.0, 0.0, 0.64, 0.76],
                          "fridge" : [4.82, -0.23, 0.0, 0.0, 0.99, -0.02],
                          "couch1" : [3.00, 1.10, 0.0, 0.0, 0.88, 0.47],
                          "couch2" : [0.52, 1.10, 0.0, 0.0, 0.45, 0.89],
                          "receptionentrance" : [0.06, -0.18, 0.0, 0.0, 0.99, 0.12],
                          "inspectionpoint" : [2.0, -2.33, 0.0, 0.0, -0.83, 0.54],
                          "entranceinspection" : [-1.38, 0.13, 0.0, 0.0, -0.05, 0.99]
                          }
        #NOTE: Assume self.params is a list of strings the first element is the name of the node to navigate to
        rospy.loginfo('Going to ' + " ".join(self.params) + ' ...')

        if "r" in self.params[0] and self.params[1] in self.room_dict:
                self.coordinates = self.room_dict[self.params[1]]
        else: 
            self.room = self.obj_dict[self.params[0]]
            self.coordinates = self.room_dict[self.room]

        print(self.params)
        print(self.coordinates)
        if len(self.params) < 1:
            rospy.logwarn("Wrong use of action, pass the coordinates the robots needs to reach in /map frame as X_Y_Theta")
        else:
            self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
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

    def _on_goTo_done(self, goalState, result):
        print("goToRoom DONE", goalState, result)
        self.params.append("done")
        rospy.loginfo('Destination reached')

    def _stop_action(self):
        self.client.cancel_all_goals()
        self.params.append("interrupted")
        rospy.loginfo('STOPPED goto action')

    @classmethod
    def is_goal_reached(cls, params):
        #TODO also make the necessary changes to make sure this returns True when the navigation has reached the goal

        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
