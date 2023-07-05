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

class IsIntentDetected(AbstractTopicCondition):
    _topic_name = "/rasa_confirmation"

    _topic_type = Bool

    def _get_value_from_data(self, data):
        return (data.data, rospy.get_time())

    def evaluate(self, params):

        if self.last_value is not None and len(self.last_value) > 0:
            # this returns true when a sentence was received and is not onlder than a minute ago
            if (rospy.get_time() - self.last_value[1]) < 10.:
                return self.last_value[0]
        return False