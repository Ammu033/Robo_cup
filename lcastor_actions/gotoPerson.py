import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
from AbstractAction import AbstractAction
import actionlib
import math
import tf
from move_base_msgs.msg import MoveBaseAction , MoveBaseGoal
from geometry_msgs.msg import PointStamped , PoseWithCovarianceStamped , Pose2D
import numpy as np


class gotoPerson(AbstractAction):

    def __init__(self, goalhandler, params):
        super().__init__(goalhandler, params)
        self.l = tf.TransformListener()
    
    def _start_action(self):
        self.gotopersonDone = False
        rospy.loginfo('Going to ' + " ".join(self.params) + ' ...')
        if len(self.params) < 1:
            rospy.logwarn("Wrong use of action, pass the coordinates the robots needs to reach in /map frame as X_Y_Theta")
        else :
            self.l = tf.TransformListener()
            self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            rospy.loginfo("Connecting to /move_base AS...")
            self.client.wait_for_server()
            rospy.loginfo("Connected.")
            robot_pose = Pose2D()
            person_point = PointStamped()
            person_point.header.frame_id = 'xtion_depth_optical_frame'
            # person_point.header.stamp = rospy.Time().now()
            goal_msg = MoveBaseGoal()
            person_point.point.x = self.params[0]
            person_point.point.y = self.params[1]
            person_point.point.z = self.params[2]
            # print(person_position_wrt_camera)
            human_wrt_map = self.l.transformPoint('map' , person_point)
            robot_pose_data  = rospy.wait_for_message('/robot_pose' , PoseWithCovarianceStamped )
            q = (
                        robot_pose_data.pose.pose.orientation.x,
                        robot_pose_data.pose.pose.orientation.y,
                        robot_pose_data.pose.pose.orientation.z,
                        robot_pose_data.pose.pose.orientation.w
                    )

            m = tf.transformations.quaternion_matrix(q)

            robot_pose.x = robot_pose_data.pose.pose.position.x
            robot_pose.y = robot_pose_data.pose.pose.position.y
            robot_pose.theta = tf.transformations.euler_from_matrix(m)[2]
            person_pos = np.array([human_wrt_map.point.x , human_wrt_map.point.y])
            print(person_pos)
            robot_pos = np.array([robot_pose.x , robot_pose.y])
            # print(person_pos)
            vector_to_person = person_pos - robot_pos

            # Calculate the distance from the robot to the person
            distance_to_person = math.sqrt(vector_to_person[0]**2 + vector_to_person[1]**2)

            # Normalize the vector to the desired distance
            normalized_vector = vector_to_person / distance_to_person

            # Calculate the orientation needed to reach the person
            goal_orientation = math.atan2(normalized_vector[1], normalized_vector[0])

            # Calculate the goal position based on the desired distance
            goal_position = person_pos - 1.2 * normalized_vector

            # return goal_position, goal_orientation    
            goal_msg.target_pose.header.stamp = rospy.Time.now()
            goal_msg.target_pose.header.frame_id = "map"
            goal_msg.target_pose.pose.position.x = goal_position[0]
            goal_msg.target_pose.pose.position.y = goal_position[1]
            print(goal_position)
            goal_msg.target_pose.pose.orientation.z = math.sin(goal_orientation/2)
            goal_msg.target_pose.pose.orientation.w = math.cos(goal_orientation/2)
            # config = goal_tolerance_client.update_configuration(people_config)
            # time.sleep(1.0)
            self.client.send_goal(goal_msg , done_cb=self.on_goto_done)
            while True:
                if self.gotopersonDone:
                    break
    def on_goto_done(self, goalState, result):
        print(result.status)
        rospy.set_param('/reached_person' , result.status)
        self.gotopersonDone = True

    def _stop_action(self):
        self.client.cancel_all_goals()
        self.params.append("interrupted")
        rospy.loginfo('STOPPED goto action')
    @classmethod
    def is_goal_reached(cls, params):
        #TODO also make the necessary changes to make sure this returns True when the navigation has reached the goal

        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
                
