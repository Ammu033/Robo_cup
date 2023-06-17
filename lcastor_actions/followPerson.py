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
class followPerson(AbstractAction):

    def _start_action(self):
        #NOTE: Assume self.params is a list of strings where the first element contains the tracker ID of the person to follow
        rospy.loginfo('Starting to follow ' + " ".join(self.params) + ' ...')

        #TODO: here put the code necessary to start the person following

    def _stop_action(self):
        #TODO: here put the code necessary to cleanly stop the person following

        self.params.append("done")
        rospy.loginfo('STOPPED follow person action')

    @classmethod
    def is_goal_reached(cls, params):        
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
