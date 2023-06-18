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
class waitOpenDoor(AbstractAction):

    def _start_action(self):
        rospy.loginfo('Waiting until the door opens...')

        #TODO: here put the code necessary to detect the opening of a door just in front of the robot

    def _stop_action(self):
        #TODO: here put the code necessary to cleanly stop the opening door detection
        

        self.params.append("done")
        rospy.loginfo('STOPPED waitopendoor action')

    @classmethod
    def is_goal_reached(cls, params):
        #TODO make the necessary changes to make sure that the below is also True after the door has been opened
        
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
