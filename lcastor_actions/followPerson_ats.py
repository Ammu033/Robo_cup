import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D
from people_msgs.msg import People
import tf
import math
import actionlib
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import message_filters
from AbstractAction import AbstractAction


DES_DIST = 1 # [m] #FIXME: to convert into a rosparam
CANCEL_DISTANCE = 0.75 # [m] #FIXME: to convert into a rosparam
ROSPARAM = '/lcastor_person_follower/personID_to_follow'

"""
Starts and stops the object detection node
"""
class followPerson_ats(AbstractAction):

    def _start_action(self):
        rospy.loginfo('Starting to follow ' + " ".join(self.params) + ' ...')
        
        self.personToFollowPos = None
        self.robot_pose = Pose2D()
        self.robot_pos_at_send_goal = None
        
        # Client init
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        # Robot pose and People subscribers
        self.sub_robot_pose = message_filters.Subscriber('/robot_pose', PoseWithCovarianceStamped)
        self.sub_people = message_filters.Subscriber('/people_tracker/people', People)
        self.ats = message_filters.ApproximateTimeSynchronizer([self.sub_robot_pose, self.sub_people], 1, 0.5)
        self.ats.registerCallback(self.cb_ats)
        
        
    def cb_ats(self, r: PoseWithCovarianceStamped, p: People):
        self.get_robot_2DPose(r)
        
        if self.robot_pos_at_send_goal is not None and self.client.get_state() == actionlib.GoalStatus.ACTIVE:
            travelled_dist = math.dist(self.robot_pos_at_send_goal, [self.robot_pose.x, self.robot_pose.y])
            rospy.logwarn("TRAVELLED DISTANCE = " + str(travelled_dist))
            if travelled_dist > CANCEL_DISTANCE: 
                rospy.logerr("GOAL CANCELLED")
                self.client.cancel_goal()
                self.robot_pos_at_send_goal = None
                
        elif self.client.get_state() != actionlib.GoalStatus.ACTIVE:
            # if rosparam is '' then return
            personID = rospy.get_param(ROSPARAM)
            if ROSPARAM == '': return
                
            # Get position of the person to follow
            personToFollowPos = self.get_person_pos(p, personID)
            if personToFollowPos is not None:
                # Follow the person
                rospy.logerr("GOAL SENT")
                self.follow_person(personToFollowPos)

        
    
    # def cb_people(self, p: People):
    #     # if goal already assigned then return
    #     if self.client.get_state() == actionlib.GoalStatus.ACTIVE: return 
        
    #     # if rosparam is '' then return
    #     personID = rospy.get_param(ROSPARAM)
    #     if ROSPARAM == '': return
            
    #     # Get position of the person to follow
    #     personToFollowPos = self.get_person_pos(p, personID)
    #     if personToFollowPos is not None:
    #         # Follow the person
    #         rospy.logerr("GOAL SENT FIRST TIME")
    #         self.follow_person(personToFollowPos)


    def get_robot_2DPose(self, r: PoseWithCovarianceStamped):
        """
        from 3D to 2D robot pose

        Args:
            p (PoseWithCovarianceStamped): 3D robot pose
        """
        
        q = (
            r.pose.pose.orientation.x,
            r.pose.pose.orientation.y,
            r.pose.pose.orientation.z,
            r.pose.pose.orientation.w
        )
        
        m = tf.transformations.quaternion_matrix(q)
        
        self.robot_pose.x = r.pose.pose.position.x
        self.robot_pose.y = r.pose.pose.position.y
        self.robot_pose.theta = tf.transformations.euler_from_matrix(m)[2]


    def get_person_pos(self, data: People, personToFollow):
        personToFollowPos = None
        for person in data.people:
            if person.name == personToFollow:
                personToFollowPos = [person.position.x, person.position.y]
        return personToFollowPos
        
        
    def calculate_goal(self, personToFollowPos):
        """
        Calculates goal pos and orientation

        Args:
            person_pos (list): [x, y]

        Returns:
            array, float: goal pos and orientation
        """
        person_pos = np.array(personToFollowPos)
        robot_pos = np.array([self.robot_pose.x, self.robot_pose.y])
        
        # Calculate the vector from the robot to the person
        vector_to_person = person_pos - robot_pos

        # Calculate the distance from the robot to the person
        distance_to_person = math.sqrt(vector_to_person[0]**2 + vector_to_person[1]**2)

        # Normalize the vector to the desired distance
        normalized_vector = vector_to_person / distance_to_person

        # Calculate the orientation needed to reach the person
        goal_orientation = math.atan2(normalized_vector[1], normalized_vector[0])

        # Calculate the goal position based on the desired distance
        goal_position = person_pos - DES_DIST * normalized_vector

        return goal_position, goal_orientation    
    
        
    def send_goal(self, goal_position, goal_orientation):
        """
        Creates goal msg

        Args:
            goal_position (array): x, y
            goal_orientation (float): theta
        """
        
        # Publish the goal position and orientation to the navigation system      
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = "map"
        goal_msg.target_pose.header.stamp = rospy.Time.now()
        goal_msg.target_pose.pose.position.x = goal_position[0]
        goal_msg.target_pose.pose.position.y = goal_position[1]
        goal_msg.target_pose.pose.orientation.z = math.sin(goal_orientation / 2)
        goal_msg.target_pose.pose.orientation.w = math.cos(goal_orientation / 2)

        self.client.send_goal(goal_msg)
        self.client.wait_for_result()
    
    
    def follow_person(self, personToFollowPos):
        """
        Calculates and publishes the goal position and orientation when personID_to_follow is in people list
        """

        # Calculate the goal position and orientation based on the desired distance
        goal_position, goal_orientation = self.calculate_goal(personToFollowPos)

        # Send the goal position and orientation to the navigation system
        self.robot_pos_at_send_goal = [self.robot_pose.x, self.robot_pose.y]
        self.send_goal(goal_position, goal_orientation)


    def _stop_action(self):
        self.sub_robot_pose.sub.unregister()
        self.sub_people.sub.unregister()
        if self.client is not None: self.client.cancel_all_goals()
        
        self.params.append("done")
        rospy.loginfo('STOPPED follow person action')
        

    @classmethod
    def is_goal_reached(cls, params):        
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
