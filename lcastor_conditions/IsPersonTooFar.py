import math
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
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
from AbstractTopicCondition import AbstractTopicCondition

ROSPARAM = '/lcastor_person_follower/personID_to_follow'
DIST_FOLLOW_THRESH = 3 # [m] #FIXME: to be set as rosparam

class IsPersonTooFar(AbstractTopicCondition):
    _topic_name = "/people_tracker/people"

    _topic_type = People
    
    
    def __init__(self):
        super(IsPersonTooFar, self).__init__()
        rospy.Subscriber('/robot_pose', PoseWithCovarianceStamped, callback = self.cb_robot_pose)
        self.robot_pose = None


    def cb_robot_pose(self, p: PoseWithCovarianceStamped):
        self.robot_pose = [p.pose.pose.position.x, p.pose.pose.position.y]
        

    def _get_value_from_data(self, data):
        return data.people
            

    def evaluate(self, params):
        
        # if rosparam is '' then just return True
        personID = rospy.get_param(ROSPARAM)
        if personID == '': return False

        if self.robot_pose is None: return False
        # if personID_to_follow is contained in the people msg then 
        # return False since the person is not lost. Otherwise return True
        for person in self.last_value:
            if person.name == personID:
                dist = math.dist(self.robot_pose, [person.position.x, person.position.y])
                return dist > DIST_FOLLOW_THRESH 
            
        return False