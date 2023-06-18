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
class listen(AbstractAction):

    def _start_action(self):
        #NOTE: Assume self.params is a list of strings...it can be used to pass the rasa "story" or whatever you thing it is useful to specify what we expect to hear at any time we start to listen
        rospy.loginfo('listenin about ' + " ".join(self.params) + ' ...')

        #TODO: here put the code necessary to start the listening node

    def _stop_action(self):
        #TODO: here put the code necessary to cleanly stop the listening node (must release all resources)
        

        self.params.append("done")
        rospy.loginfo('STOPPED listen action')

    @classmethod
    def is_goal_reached(cls, params):
        #TODO also make the necessary changes to make sure this returns True when a sentence has been listened
        
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
