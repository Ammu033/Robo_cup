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
class personIdentification(AbstractAction):

    def _start_action(self):
        #NOTE: Assume self.params is a list of strings which contains the type of identifications we want to start (i.e., face, tshirt, hair... ) 
        rospy.loginfo('Starting person identification node ' + " ".join(self.params) + ' ...')

        
        if self.params[0] == "learn":
            rospy.set_param('learn' , 1)
        elif self.params[0] == "eval":
            rospy.set_param('learn' , 0)
        self._stop_action()
        



        #TODO: here put the code necessary to start the person identification node so that it's ready to receive identification requests

    def _stop_action(self):
        #TODO: here put the code necessary to stop the person identification node (must clean all resources)
        

        self.params.append("done")
        rospy.loginfo('STOPPED personIdentification action')

    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
