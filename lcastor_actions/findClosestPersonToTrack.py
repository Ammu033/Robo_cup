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
class findClosestPersonToTrack(AbstractAction):

    def _start_action(self):
        rospy.loginfo('Starting to finding person to track ' + " ".join(self.params) + ' ...')

        #TODO: here put the code necessary that:
        #     1. waits until there is one person standing in front of the robot for at least N second and not further than 1.5 metres (to avoid false positives)
        #     2. takes the tracking ID of the person and saves it in a ros param

    def _stop_action(self):
        #TODO: here put the code necessary to clean all the resources used

        self.params.append("done")
        rospy.loginfo('STOPPED listen action')

    @classmethod
    def is_goal_reached(cls, params):
        #TODO make sure the below returns True once a person to track has been found
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
