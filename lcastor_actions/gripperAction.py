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
class gripperAction(AbstractAction):

    def _start_action(self):
        #NOTE: Assume self.params is a list of strings where the first element contains either "open" or "close"
        rospy.loginfo('Starting to ' + " ".join(self.params) + ' gripper ...')

        #TODO: here put the code necessary to open/close the gripper
        if self.params[0] == "open":
            #TODO
        elif self.params[0] == "close":
            #TODO
        else:
            rospy.logwarn("Gripper action {} not recognised!".format(self.params[0]))

    def _stop_action(self):
        #TODO: here put the code necessary to cleanly stop the gripper action

        self.params.append("done")
        rospy.loginfo('STOPPED move gripper action')

    @classmethod
    def is_goal_reached(cls, params):
        #TODO make sure the below returns True when the gripper action has been execyted fully
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
