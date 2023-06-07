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



from std_msgs.msg import String
from AbstractTopicCondition import AbstractTopicCondition

class test(AbstractTopicCondition):
    _topic_name = "/map_in_use"

    _topic_type = String

    def _get_value_from_data(self, data):
        return (str(data.data), rospy.Time.now())

    def evaluate(self, params):
        if self.last_value is not None:
            if (self.last_value != str(params[0])) and\
                    ((rospy.Time.now() - self.last_value[1]) < rospy.Duration.from_sec(5)):
                return True
        return False