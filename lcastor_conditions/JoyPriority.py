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

from std_msgs.msg import String
from AbstractTopicCondition import AbstractTopicCondition

class JoyPriority(AbstractTopicCondition):
    _topic_name = "/joy_priority"

    _topic_type = Bool

    def _get_value_from_data(self, data):
        return str(data.data)

    def evaluate(self, params):
        if self.last_value is not None:
            if (self.last_value.lower() == str(params[0]).lower()):
                return True
        return False