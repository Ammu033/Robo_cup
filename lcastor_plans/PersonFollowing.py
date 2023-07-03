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
        
    # p.exec_action('setNavigationMode', 'MAP')
    
    # p.action_cmd('getYesNoConfirmation', 'Have_we_arrived?', 'start')

    while(not p.get_condition("IsYesConfirmed")):
        p.exec_action('speak', 'Can_you_stand_in_front_of_me,_please?')
        # p.exec_action('speak', 'Can_any_of_you_stand_in_front_of_me,_please?')
        
        p.exec_action('findClosestPersonToTrack', '')

        p.exec_action('speak', "Hey_person_" + str(rospy.get_param(ROSPARAM)) + ",_I_am_following_you")
        p.exec_action('speak', 'Please confirm on my display when we arrive to the destination')
        
        p.action_cmd('followPerson', '', 'start')
        
        display_notification = time.time()
        dist_notification = time.time()
        while not p.get_condition("IsYesConfirmed") and not p.get_condition("IsPersonLost"):
            
            # Please confirm task finished notification
            t_display = time.time() - display_notification
            if (t_display > 2*SPEAK_TIMEOUT):
                display_notification = time.time()
                p.exec_action('speak', 'Please confirm on my display when we arrive to the destination')
            
            # IsPersonTooFar notification
            t_dist = time.time() - dist_notification
            if p.get_condition("IsPersonTooFar") and (t_dist > SPEAK_TIMEOUT):
                dist_notification = time.time()
                p.exec_action('speak', 'Can_you_slow_down,_please?')
                
            time.sleep(0.1)
        
        p.action_cmd('followPerson', '', 'stop')
        if not p.get_condition("IsYesConfirmed"): p.exec_action('speak', 'Person_to_follow_lost')
        
    # # Go back to the initial position
    # p.action_cmd('getYesNoConfirmation', '', 'stop')
    # p.exec_action('goto', '0.0_0.0_0.0')
    
    # # Changing navigation mode to LOC
    # p.exec_action('setNavigationMode', 'LOC')


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    PersonFollowing(p)

    p.end()
