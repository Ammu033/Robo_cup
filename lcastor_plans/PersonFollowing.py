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
        time.sleep(2)
        p.exec_action('findClosestPersonToTrack', '')

        p.exec_action('speak', "Hey_you,_I_am_following_you._Please_start_moving.")
        # p.exec_action('speak', "Hey_person_" + str(rospy.get_param(ROSPARAM)) + ",_I_am_following_you")
        
        p.action_cmd('followPerson', '', 'start')
        
        dist_notification = time.time()

        while not taskFinished and not p.get_condition("IsPersonLost"):
            
            # IsPersonTooFar notification
            t_dist = time.time() - dist_notification
            if p.get_condition("IsPersonTooFar") and (t_dist > SPEAK_TIMEOUT):
                dist_notification = time.time()
                p.exec_action('speak', 'Can_you_slow_down,_please?')
                
            time.sleep(5)
            
        p.action_cmd('followPerson', '', 'stop')
        # p.exec_action('speak', 'Person_to_follow_lost')
        # if rospy.get_time() - starting_time > 30:
            # start = rospy.get_time()
            # while msg_missed:
                # if rospy.get_time() - start > 30:
                    # break
                # p.exec_action("speak", 'Have_we_arrived?_Please_come_to_me,_and_say_yes_or_no!')
                # p.exec_action("activateRasa", "affirm_deny")
                # try:
                    # taskFinished = rospy.wait_for_message('/person_affirm_deny', Bool, timeout = QUESTION_TIMEOUT).data
                    # msg_missed = False
                # except:
                    # msg_missed = True
                




if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    PersonFollowing(p)

    p.end()
