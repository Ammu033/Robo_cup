import os
import sys
import rospy
try:
    rospy.logwarn("asdsd")
    sys.path.insert(0, os.environ["PNP_HOME"] + '/conditions')
    rospy.logwarn("sys path inserted")
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    rospy.logwarn("Error inserting sys path")
    sys.exit(1)

from std_msgs.msg import Bool

from AbstractTopicCondition import AbstractTopicCondition

class IsGestureDetected(AbstractTopicCondition):
    _topic_name = "/test" #TODO change this with the correct topic

    _topic_type = Bool #TODO change this with the correct topic message

    def _get_value_from_data(self, data):
        # NOTE data is the message received from the topic specified above, what you return here will go into self.last_value
        # TODO this should return a list of all the gesture recognized
        return #TODO

    def evaluate(self, params):
        #NOTE assume params is a list of strings, the first element is the gesture we are interested in
        #TODO this should return True if the gesture has been detected, False otherwise
        return #TODO