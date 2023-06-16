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
class goto(AbstractAction):

    def _start_action(self):
        #NOTE: Assume self.params is a list of strings the first element is the name of the node to navigate to
        rospy.loginfo('Going to ' + " ".join(self.params) + ' ...')

        #TODO: here put the code necessary to navigate to the node 

    def _stop_action(self):
        #TODO: here put the code necessary to cleanly stop the navigation
        

        self.params.append("done")
        rospy.loginfo('STOPPED detect action')

    @classmethod
    def is_goal_reached(cls, params):
        #TODO also make the necessary changes to make sure this returns True when the navigation has reached the goal

        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
