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
from sensor_msgs.msg import LaserScan
from AbstractTopicCondition import AbstractTopicCondition

class isDoorOpen(AbstractTopicCondition):
    _topic_name = "/scan"
    _topic_type = LaserScan


    def _get_value_from_data(self, data):
        front_index = len(data.ranges) // 2  # Assuming front index is at the center of the ranges
        distance_threshold = 2.0  # Minimum distance threshold for the door to be considered open

        if data.ranges[front_index] > distance_threshold:
            door_open = True
        else:
            door_open = False

        return bool(door_open)

    def evaluate(self, params):
        return self.last_value