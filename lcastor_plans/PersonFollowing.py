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

SPEAK_TIMEOUT = 5 # [s]
ROSPARAM = 'lcastor_person_follower/personID_to_follow'

def PersonFollowing(p):
    rospy.set_param(ROSPARAM, '')
    
    while(not p.get_condition("IsCarReached")):
        p.exec_action('speak', 'Can_any_of_you_stand_in_front_of_me,_please?')
        
        p.exec_action('findClosestPersonToTrack', '')
        p.exec_action('speak', "Hey_person_" + str(rospy.get_param(ROSPARAM)) + ",_I_am_following_you")
        
        p.action_cmd('followPerson', '', 'start')
        
        dist_notification = time.time()
        while not p.get_condition("IsCarReached") and not p.get_condition("IsPersonLost"):
            # time check is needed in order to not say "slow down" 
            # at each instant the condition isPersonTooFar is True
            t = time.time() - dist_notification
            if p.get_condition("IsPersonTooFar") and (t > SPEAK_TIMEOUT):
                dist_notification = time.time()
                p.exec_action('speak', 'Can_you_slow_down,_please?')
            time.sleep(0.1)
        
        p.action_cmd('followPerson', '', 'stop')
        p.exec_action('speak', 'Person_to_follow_lost')
        
    # Go back to the initial position
    p.exec_action('goto', '0.0_0.0_0.0')


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    PersonFollowing(p)

    p.end()
