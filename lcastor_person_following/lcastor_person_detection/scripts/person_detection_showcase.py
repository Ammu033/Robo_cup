#!/usr/bin/env python

import rospy
from people_msgs.msg import People
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D
from std_msgs.msg import String, Bool
import tf
import math
import actionlib
from pal_interaction_msgs.msg import TtsActionGoal, TtsAction, TtsGoal
import message_filters


NODE_NAME = 'lcastor_person_detection'
NODE_RATE = 100 # [Hz]
SPEAK_TIMEOUT = 15.0 # [s]
# FIELD_OF_VIEW = 60 # [°]
# DIST_THRESH = 3 # [m]
FIELD_OF_VIEW = float(rospy.get_param("/lcastor_person_detection/field_of_view"))
DIST_DETECTION_THRESH = float(rospy.get_param("/lcastor_person_detection/dist_detection_threshold"))
DIST_FOLLOW_THRESH = float(rospy.get_param("/lcastor_person_detection/dist_follow_threshold"))




def speak(msg):
    ac = actionlib.SimpleActionClient("/tts", TtsAction)
    ac.wait_for_server()

    # create goal
    goal = TtsActionGoal()
    goal.goal.rawtext.text = msg
    goal.goal.rawtext.lang_id = "en_GB"

    # send goal
    ac.send_goal(goal.goal)
    ac.wait_for_result()


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


class PersonDetector():
    """
    Person Detector class
    """
    
    def __init__(self):
        """
        Class constructor. Init publishers and subscribers
        """
        
        self.robot_pose = Pose2D()
        self.person_to_follow = None
        self.lost = True
        self.dist_notification = rospy.Time.now()
        self.person_follower_active = False
        self.username = None
        
        # Robot pose subscriber
        self.sub_robot_pos = rospy.Subscriber('/robot_pose', PoseWithCovarianceStamped, callback = self.cb_robot_pose)
        
        # People subscriber
        self.sub_person_pos = rospy.Subscriber('/people_tracker/people', People, callback = self.cb_people)
        
        # Person to follow publisher
        self.pub_personID_to_follow = rospy.Publisher('/person_to_follow', String, queue_size = 10)
        
        self.sub_trigger = rospy.Subscriber('/nlp/trigger', Bool, callback = self.cb_trigger) # FIXME: [showcase]
        self.sub_username = rospy.Subscriber('/nlp/username', String, callback = self.cb_username) # FIXME: [showcase]
        
    # FIXME: [showcase]
    def cb_username(self, msg : String):
        self.username = msg.data
        self.say_intro()
        
            
    # FIXME: [showcase]
    def cb_trigger(self, msg : Bool):
        self.person_follower_active = msg.data
        
        
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
        self.person_lost(data.people)
        if self.username is not None and self.person_follower_active:
            if not self.lost: self.check_distance(data.people)
            if self.robot_pose.x is not None and self.lost:
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
                    rospy.logwarn("Person ID to follow: " + self.person_to_follow.name)
                    self.pub_personID_to_follow.publish(self.person_to_follow.name)
                    self.say_person_detected()
                
                
    def person_lost(self, people):
        if self.person_to_follow is None: 
            self.lost = True
            return
        for person in people:
            if person.name == self.person_to_follow.name: 
                self.lost = False
                return
        self.say_person_lost()
        self.person_to_follow = None
        self.lost = True
        self.username = None # FIXME: [showcase]


    def check_distance(self, people):
        for person in people:
            if person.name == self.person_to_follow.name: 
                dist = math.dist([self.robot_pose.x, self.robot_pose.y],[person.position.x, person.position.y])
                rospy.logwarn(dist)
                t = rospy.Time.now() - self.dist_notification
                if dist > DIST_FOLLOW_THRESH and (t.to_sec() > SPEAK_TIMEOUT):
                    self.dist_notification = rospy.Time.now()
                    self.say_slowdown()
        
    # def say_no_person_detected(self, event):
    #     if self.person_to_follow is None: 
    #         speak("No person to follow detected")


    def say_slowdown(self):
        speak("Can you slow down, please?")
            

    def say_intro(self):
        speak(self.username + ", can you please stand in front of me?") # FIXME: [showcase]
        # speak("Can any of you stand in front of me, please?")
        
        
    def say_person_detected(self):
        speak(self.username + ", I am going to follow you") # FIXME: [showcase]
        # speak("Person ID to follow: " + self.person_to_follow.name)
    
    
    def say_person_lost(self):
        speak(self.username + ", I have lost you") # FIXME: [showcase]
        # speak("Person to follow lost")

            
if __name__ == '__main__':    
        
    # Init node
    rospy.init_node(NODE_NAME)

    # Set node rate
    rate = rospy.Rate(NODE_RATE)
    person_detector = PersonDetector()
    # person_detector.say_intro() # FIXME: [showcase]
    
    # interval = rospy.Duration(SPEAK_TIMEOUT)
    # timer = rospy.Timer(interval, person_detector.say_no_person_detected)
    
    while not rospy.is_shutdown():
        rate.sleep()
