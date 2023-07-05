import os
import rospy
import sys
from robocup_human_sensing.msg import RegionOfInterest2D
from std_msgs.msg import UInt64
from queue import Queue
try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/conditions')
    rospy.logwarn("sys path inserted")
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    rospy.logwarn("Error inserting sys path")
    sys.exit(1)

from AbstractTopicCondition import AbstractTopicCondition

class IsPersonInRange(AbstractTopicCondition):
    _topic_name = '/closestPersonDistance'
    _topic_type = UInt64

    def _get_value_from_data(self, data):
        if self.last_value is None or len(self.last_value) ==0 :
            return [[], [], [], data.data]
        l = list(self.last_value)
        l.pop(0)
        l.append(data.ids)
        return l
    
    def evaluate(self , params):
        if len(params) > 0:
            thres = float(params[0]) * 1000  # Pass the Required threshold in Meters
            #print(id_to_check)
            condition = True
            for i in range(4):
                if self.last_value[i] < thres:
                    condition = condition and True
                else :
                    condition = condition and False
                #condition = (thres in self.last_value[i]) and condition
        else:
            rospy.WARN('You have not passed the Params into the condition " Threshold " ')
        return condition