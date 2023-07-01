import os
import sys
import tf
import math

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
from people_msgs.msg import People
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D
from AbstractAction import AbstractAction


FIELD_OF_VIEW = 60 # [°] #FIXME: to convert into a rosparam
DIST_DETECTION_THRESH = 3 # [m] #FIXME: to convert into a rosparam
ROSPARAM = '/lcastor_person_follower/personID_to_follow'

def wrapToPi(angle):
    """
    Adjust the angle to be within the range of -pi to pi

    Args:
        angle (float): Angle in rads

    Returns:
        float: Angle in rads mapped within the range of -pi to pi
    """
    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle < -math.pi:
        angle += 2 * math.pi
    return angle

"""
Starts and stops the object detection node
"""
class findClosestPersonToTrack(AbstractAction):

    def _start_action(self):
        rospy.loginfo('Starting to finding person to track ' + " ".join(self.params) + ' ...')
        
        self.robot_pose = Pose2D()
        self.person_to_follow = None
        
        # Robot pose and People subscriber
        self.sub_robot_pose = rospy.Subscriber('/robot_pose', PoseWithCovarianceStamped, callback = self.cb_robot_pose)
        self.sub_people = rospy.Subscriber('/people_tracker/people', People, callback = self.cb_people)
                
        rospy.loginfo("Waiting for personID to track...")

        
    def cb_robot_pose(self, p: PoseWithCovarianceStamped):
        """
        from 3D to 2D robot pose

        Args:
            p (PoseWithCovarianceStamped): 3D robot pose
        """
        
        q = (
            p.pose.pose.orientation.x,
            p.pose.pose.orientation.y,
            p.pose.pose.orientation.z,
            p.pose.pose.orientation.w
        )
        
        m = tf.transformations.quaternion_matrix(q)
        
        self.robot_pose.x = p.pose.pose.position.x
        self.robot_pose.y = p.pose.pose.position.y
        self.robot_pose.theta = tf.transformations.euler_from_matrix(m)[2]
        
    
    def cb_people(self, data: People):
        """
        Handles the people topic from the tracker and identifies the person to follow

        Args:
            data (People): people topic from the tracker
        """
        if self.robot_pose.x is not None:
            closest_distance = float('inf')  # Initialize with a large value
                
            for person in data.people:
                # Calculate distance (assuming position is in meters)
                distance = math.dist([self.robot_pose.x, self.robot_pose.y], [person.position.x, person.position.y])
                    
                # Calculate angle between person and robot
                angle = math.atan2(person.position.y - self.robot_pose.y, person.position.x - self.robot_pose.x)
                    
                # Convert the angle to robot-centric coordinates
                relative_angle = wrapToPi(angle - self.robot_pose.theta)
                    
                # Check if the person is within the desired field of view angle range
                if abs(relative_angle) <= math.radians(FIELD_OF_VIEW / 2) and distance <= DIST_DETECTION_THRESH:
                    if distance < closest_distance:
                        closest_distance = distance
                        self.person_to_follow = person
                        
            if self.person_to_follow is not None:
                self._stop_action()

        

    def _stop_action(self):
        self.sub_robot_pose.unregister()
        self.sub_people.unregister()
        rospy.loginfo("personID to follow = " + self.person_to_follow.name)
        rospy.set_param(ROSPARAM, self.person_to_follow.name)
        self.params.append("done")
        
        rospy.loginfo('STOPPED findClosestPersonToTrack action')


    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
