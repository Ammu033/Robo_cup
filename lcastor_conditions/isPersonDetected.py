import os
import rospy
import sys
from robocup_human_sensing.msg import RegionOfInterest2D
from queue import Queue
try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/conditions')
    rospy.logwarn("sys path inserted")
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    rospy.logwarn("Error inserting sys path")
    sys.exit(1)

from AbstractTopicCondition import AbstractTopicCondition

class isPersonDetected(AbstractTopicCondition):
    _topic_name = '/h_boxes_tracked'
    _topic_type = RegionOfInterest2D
    
    # q = Queue(maxsize = 4)
    # def check_frames(self):
    #     while not self.q.full:
            
    def _get_value_from_data(self, data):
        if self.last_value is None or len(self.last_value) ==0 :
            return [[], [], [], data.ids]
        l = list(self.last_value)
        l.pop(0)
        l.append(data.ids)
        return l
    
    def evaluate(self , params):
        if self.last_value is not None:
            if len(params) > 0:
                id_to_check = rospy.get_param(params[0]+"/id")
                print(id_to_check)
                condition = True
                for i in range(4):
                    condition = (id_to_check in self.last_value[i]) and condition
            else:
                condition = True
                for i in range(4):
                    condition = (len(self.last_value[i]) > 0) and condition
            return condition
        else:
            return False