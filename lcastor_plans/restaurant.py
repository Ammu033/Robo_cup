import os
import sys
import rospy

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import time
import pnp_cmd_ros
from pnp_cmd_ros import *
# from wait_for_person import wait_for_person
# from identify_person import identify_person
# from obtain_person_information import obtain_person_information
# from introduce_people import introduce_people
# from do_guest import do_guest
from find_people import find_people
import time
from get_order import get_order
import pnp_cmd_ros
from pnp_cmd_ros import *
from move_base_msgs.msg import MoveBaseAction , MoveBaseGoal
from geometry_msgs.msg import PointStamped , PoseWithCovarianceStamped , Pose2D
import math
import numpy as np
import tf
import actionlib
from dynamic_reconfigure.client import Client
import time
DES_DIST = 1.2
l = None
rotation_angle = None
person_point = None
robot_pose = None
goal_msg = None
client = None
goal_tolerance_client = None
gotopersonDone = False
people_config = None
other_config = None



def callback(config):
    rospy.loginfo("Config set to: {xy_goal_tolerance}".format(**config))

# bar_location = 0.0 , 0.0 , 0.0  , 0.0 # NEED TO FIX THIS WITH CORRECT FORMAT OF THE LOCATION             
def restaurant(p):
    global rotation_angle
    # rospy.set_param(')
    rotation_angle = 0
    number_of_customers_served = 0
    while number_of_customers_served  < 2:
        rospy.set_param('found_person' , False)
        while not rospy.get_param('/found_person', False) :
            find_people(p,rotation_angle)
            rotation_angle +=(np.pi/3) #NEED TO CHANGE BASED ON OBSERVATION
        p.exec_action('speak', 'I_detected_a_waving_person')
        guest_location = [rospy.get_param('/person_location/x'),
                            rospy.get_param('/person_location/y'),
                            rospy.get_param('/person_location/z')] #NEED TO FIX WITH MOVE_BASE TOLERANCE
        # p.exec_action('goto', guest_location )   # NEED TO FIX THIS WITH CORRECT FORMAT OF THE LOCATION             
        goto_person(guest_location)
        list_of_items = get_order(p , from_guest = True  ) # FIX RETURN FROM FUNCGTION
        # p.exec_action('goto' , bar_location)
        p.exec_action('goto', '0.0_0.0_0.0') # BAR LOCATION
        get_order(p , to_barman = True  ,list_of_items= list_of_items)
        goto_person(guest_location)
        # p.exec_action('goto', guest_location)   # NEED TO FIX THIS WITH CORRECT FORMAT OF THE LOCATION             
        get_order(p , to_guest = True)
        # p.exec_action('goto' , bar_location)
        p.exec_action('goto', '0.0_0.0_0.0') # BAR LOCATION

        number_of_customers_served += 1


def goto_person(person_position_wrt_camera ):
    global goal_msg, robot_pose, person_point
    person_point.point.x = person_position_wrt_camera[0]
    person_point.point.y = person_position_wrt_camera[1]
    person_point.point.z = person_position_wrt_camera[2]
    print(person_position_wrt_camera)
    human_wrt_map = l.transformPoint('map' , person_point)
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
    goal_position = person_pos - DES_DIST * normalized_vector

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
    client.send_goal(goal_msg , done_cb=on_goto_done)
    while True:
        if gotopersonDone:
            break

def on_goto_done(_,__):
    global gotopersonDone
    print('Go To Command Successfully Sent')
    gotopersonDone = True

if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    l = tf.TransformListener() 
    rotation_angle = 0.0
    person_point = PointStamped()
    person_point.header.frame_id = 'xtion_depth_optical_frame'
    robot_pose = Pose2D()
    goal_msg = MoveBaseGoal()
    client = actionlib.SimpleActionClient('/move_base' , MoveBaseAction)
    # goal_tolerance_client = Client("/move_base/TebLocalPlannerROS/", timeout=30, config_callback=callback)
    # client.wait_for_server()
    # people_config = {"xy_goal_tolerance": 1.0} 
    # other_config = {"xy_goal_tolerance": 0.2}

    restaurant(p)

    p.end()