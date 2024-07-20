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
import numpy as np

def find_people(p ,  rotation_angle):
    

    # p.exec_action('moveHead' , '0.0_'+ str(rotation_angle))
    p.exec_action('goto', '0.0_0.0_'+ str(rotation_angle))
    rospy.set_param('/found_person' , False)
    # gesture_detected = False
    start_time = rospy.Time.now()
    # id_with_gesture_count = {} 
    while ((rospy.Time.now().secs - start_time.secs)  < 15) :
        print(rospy.get_param('/found_person', False))
        print('searching for the person')
        if rospy.get_param('/found_person', False) : 
            print('Found the person')
            break

        
    # print("NEED TO POPULATE")

if __name__ == "__main__":
    p = PNPCmd()
    p.begin()
    find_people(p , 0.0)
    p.end()