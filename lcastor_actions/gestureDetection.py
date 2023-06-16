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
class gestureDetection(AbstractAction):

    def _start_action(self):
        #NOTE: Assume self.params is a list of strings with the labels of the gestures we want to detect
        rospy.loginfo('Detecting gestures ' + " ".join(self.params) + ' ...')

        #TODO: here put the code necessary to start the gesture detection node

    def _stop_action(self):
        #TODO: here put the code necessary to cleanly stop the gesture detection node (must release all resources)
        

        self.params.append("done")
        rospy.loginfo('STOPPED gesture action')

    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
