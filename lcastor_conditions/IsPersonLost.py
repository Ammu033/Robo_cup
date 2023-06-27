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

from people_msgs.msg import People

from AbstractTopicCondition import AbstractTopicCondition

ROSPARAM = '/lcastor_person_follower/personID_to_follow'

class IsPersonLost(AbstractTopicCondition):
    _topic_name = "/people_tracker/people"

    _topic_type = People

    def _get_value_from_data(self, data):
        return data.people
            

    def evaluate(self, params):
        
        # if rosparam is '' then just return True
        personID = rospy.get_param(ROSPARAM)
        if personID == '': return True
        
        # if personID_to_follow is contained in the people msg then 
        # return False since the person is not lost. Otherwise return True
        for person in self.last_value:
            if person.name == personID: 
                return False
            
        # Before returning True, personID_to_follow rosparam = '' 
        rospy.set_param(ROSPARAM, '')
        return True