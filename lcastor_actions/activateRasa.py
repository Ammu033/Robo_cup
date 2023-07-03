import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
from AbstractAction import AbstractAction
import actionlib
from std_msgs.msg import String

class activateRasa(AbstractAction):

    def _start_action(self):
        rospy.loginfo('Obtaining detecting intent from user input: ' + " ".join(self.params) + ' ...')
        #self.starting_time = rospy.Time.now()

        self.input = rospy.Publisher("/planner_intention", String, queue_size=10)

        if len(self.params) > 0:
            rospy.sleep(0.5)
            self.input.publish(str("_".join(self.params)))#
        else:
            rospy.logwarn("Wrong use of activateIntent action, you should pass the intent name!")

        self.params.append("done")

    def _stop_action(self):

        rospy.loginfo('STOPPED activateIntent')

    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True

        return reached