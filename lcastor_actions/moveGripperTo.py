import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
from AbstractAction import AbstractAction

"""
Starts and stops the object detection node
"""
class moveGripperTo(AbstractAction):

    def _start_action(self):
        #NOTE: Assume self.params is a list of strings which identify the position where to move
        #TODO decide how to do this, i.e. whether we pass position and orientation coords or menmonic names for specific locations
        rospy.loginfo('Starting to move the gripper to ' + " ".join(self.params) + ' ...')

        #TODO: here put the code necessary to move the gripper to specified location

    def _stop_action(self):
        #TODO: here put the code necessary to cleanly stop the gripper action

        self.params.append("done")
        rospy.loginfo('STOPPED moving gripper')

    @classmethod
    def is_goal_reached(cls, params):
        #TODO make sure the below returns True when the gripper action has been execyted fully
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
