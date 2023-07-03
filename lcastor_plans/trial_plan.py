import os
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
def people_id(p):
    p.get_condition('isPersonDetected_guest1')
    #p.action_cmd('Recog' , "" , "start")
    rospy.set_param('personSaved' , 0)
    rospy.set_param('modelLoaded' , False)
    p.action_cmd('peopleDetection' , "" , "start")
    #p.exec_action('personIdentification' , "launch")
    while not rospy.get_param('modelLoaded'): time.sleep(0.01)

    p.exec_action('speak' , 'Can_you_stand_in_front_of_me,_please?')
    #p.exec_action('findClosestPersonToTrack' , '')
    p.exec_action('personIdentification' , 'learn' )

    while not rospy.get_param('personSaved'): time.sleep(0.01)
    p.exec_action('saveGuestData' , 'setid_guest1')
    p.exec_action('speak' , 'can_you_tell_your_name?')
    # implement rasa actions and calling name and drink 
    #p.exec_action('saveGuestData' , 'setname_guest1')
    while not p.get_condition('isPersonDetected_guest1'):
        rospy.sleep(0.01)
    p.exec_action('speak' , 'guest1_detected')
    name = rospy.get_param('/guest1/name')
    p.exec_action('speak', name)
if __name__ == "__main__":
    p = PNPCmd()
    p.begin()
    people_id(p)
    p.end()