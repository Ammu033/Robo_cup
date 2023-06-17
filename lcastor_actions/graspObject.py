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
class graspObject(AbstractAction):

    def _start_action(self):
        #NOTE: Assume self.params is a list of strings where the first element contains the name of the object to grasp
        rospy.loginfo('Starting to grasp ' + " ".join(self.params) + ' ...')

        #TODO: here put the code necessary to grasp the object

    def _stop_action(self):
        #TODO: here put the code necessary to cleanly stop the object grasping

        self.params.append("done")
        rospy.loginfo('STOPPED grasp object action')

    @classmethod
    def is_goal_reached(cls, params):
        #TODO make sure the below returns True when the object has been grasped
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
