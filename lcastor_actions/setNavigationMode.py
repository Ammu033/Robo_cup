import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
from AbstractAction import AbstractAction
from pal_navigation_msgs.srv import Acknowledgment

"""
Starts and stops the object detection node
"""
class setNavigationMode(AbstractAction):

    def _start_action(self):
        rospy.loginfo('Changing Navigation Mode to ' + " ".join(self.params) + ' ...')
        
        # action which execute this command rosservice call /pal_navigation_sm "input: 'NAV_MODE'" 
        NAV_MODE = self.param[0]

        rospy.wait_for_service('/pal_navigation')
        change_navmode = rospy.ServiceProxy('/pal_navigation_sm', Acknowledgment)
        res = change_navmode.call(NAV_MODE)
        
        if res[0]:
            rospy.loginfo('Navigation Mode change successfully')
        else:
            rospy.loginfo('Navigation Mode error: ' + str(res[1]))
        
        
        self._stop_action()
        


    def _stop_action(self):
        self.params.append("done")
        rospy.loginfo('STOPPED setNavigationMode action')

    @classmethod
    def is_goal_reached(cls, params):

        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
