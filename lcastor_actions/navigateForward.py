import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
from AbstractAction import AbstractAction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math
from geometry_msgs.msg import Twist

"""
Starts and stops the object detection node
"""
nav_pub = rospy.Publisher("/nav_vel", Twist)

class navigateForward(AbstractAction):

    def _start_action(self):
        #NOTE: Assume self.params is a list of strings the first element is the name of the node to navigate to
        rospy.loginfo('Going forward for' + " ".join(self.params) + 's ...')

        if len(self.params) < 1:
            rospy.logwarn("Wrong use of action, pass how many seconds the robot needs to navigate at 0.25m/s")
        else:
            # Create target velocity msg
            self.nav_msg = Twist()
            self.nav_msg.linear.x = 0.25

            start_t = rospy.get_time()
            rospy.loginfo("Sending the robot forward...")
            while rospy.get_time() - start_t <= float(self.params[0]): nav_pub.publish(self.nav_msg)
            

    def _on_navigateForward_done(self, goalState, result):
        print("navigateForward DONE", goalState, result)
        self.params.append("done")
        rospy.loginfo('Destination reached')

    def _stop_action(self):
        self.params.append("done")
        rospy.loginfo('STOPPED navigateForward action')

    @classmethod
    def is_goal_reached(cls, params):
        #TODO also make the necessary changes to make sure this returns True when the navigation has reached the goal

        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
