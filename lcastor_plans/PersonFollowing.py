import os
import sys
import rospy
from AskConfirmation import AskConfirmation

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import time
import pnp_cmd_ros
from pnp_cmd_ros import *
from std_msgs.msg import Bool


SPEAK_TIMEOUT = 5 # [s]
ROSPARAM = 'lcastor_person_follower/personID_to_follow'
QUESTION_TIMEOUT = 10

def PersonFollowing(p):
    print('Able to start the plan')
    rospy.set_param(ROSPARAM, '')
    msg_missed = True
    
    taskFinished = False
    starting_time = rospy.get_time()
    while(not taskFinished):
        
        p.exec_action('speak', 'Can_you_stand_2_meters_in_front_of_me,_please?')
        # time.sleep(2)
        p.exec_action('findClosestPersonToTrack', '')

        p.action_cmd('followPerson', '', 'start')

        p.exec_action('speak', "Hey_you,_I_am_following_you._Please_start_moving.")
        # p.exec_action('speak', "Hey_person_" + str(rospy.get_param(ROSPARAM)) + ",_I_am_following_you")
        
        
        
        dist_notification = time.time()

        while not p.get_condition("IsPersonLost"):
            
            # IsPersonTooFar notification
            t_dist = time.time() - dist_notification
            if p.get_condition("IsPersonTooFar") and (t_dist > SPEAK_TIMEOUT):
                dist_notification = time.time()
                p.exec_action('speak', 'Can_you_slow_down,_please?')
                
            time.sleep(5)
            
        p.action_cmd('followPerson', '', 'stop')
        # p.exec_action('speak', 'Person_to_follow_lost')
        if rospy.get_time() - starting_time > 30:
            start = rospy.get_time()
            response = None
            while response is None:
                if rospy.get_time() - start > 30:
                    break
                success, response = AskConfirmation(p, 'Have_we_arrived?', "Please_confirm_if_we_have_arrived")
                if success and (response == "yes"):
                    taskFinished = True


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    PersonFollowing(p)

    p.end()
