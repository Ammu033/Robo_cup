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
from wait_for_person import wait_for_person
from identify_person import identify_person
from obtain_person_information import obtain_person_information
from introduce_people import indtroduce_people
import time
import pnp_cmd_ros
from pnp_cmd_ros import *

def Receptionist(p):
    
    rospy.set_param('personSaved' , 0)
    rospy.set_param('modelLoaded' , False)
    #p.action_cmd('peopleDetection' , "" , "start")
    #p.exec_action('personIdentification' , "launch")

    while not rospy.get_param('modelLoaded'): time.sleep(0.01)
    # 1. The robot is in a predefined location near the entrance door
    wait_for_person(p)
    # 2. ask the person to go in front of the robot to get their details
    #p.exec_action("speak", "Please_move_one_metre_in_front_of_me_so_that_I_can_see_you_better.")
    #p.exec_action("speak", "Say,_hey_tiago_continue,_once_you_have_done_so.")

    #p.exec_action("listen", "continue") # TODO this waits that someone confirms the robot can continue
    #p.get_condition('isPersonDetected_guest1')
    #p.action_cmd('Recog' , "" , "start")


    p.exec_action('speak' , 'Hi_I_am_tiago,_welcome_to_the_party!')
    p.exec_action('speak' , 'Can_you_stand_there_and_look_at_my_eyes,_please?')
    while not p.get_condition("isPersonDetected"):
        time.sleep(1)
    #p.exec_action('findClosestPersonToTrack' , '')
    p.exec_action('personIdentification' , 'learn' )
    while not rospy.get_param('personSaved'): time.sleep(0.1)
    p.exec_action('saveGuestData' , 'setid_guest1')
    #p.exec_action('speak' , 'can_you_tell_your_name?')
    #p.exec_action('activateRasa' , 'guest_name')
    obtain_person_information(p , 'guest1' , 'name')
    obtain_person_information(p , 'guest1' , 'drink')
    p.exec_action('speak' , 'Please_follow_me_to_the_party_' + rospy.get_param('guest1/name'))
    p.exec_action('gotoRoom' , 'livingroom')#TODO set location of where to go 
    indtroduce_people('host' , 'guest1')
    p.exec_action('gotoRoom' , 'r_couch')
    p.exec_action('speak' , 'Please_be_seated_on_the_couch_' + str(rospy.get_param('/guest1/name')))
    time.sleep(10)
    p.exec_action('speak' , 'I_will_now_go_to_the_entrance_and_wait_for_other_guests,_feel_at_home!!!')


    p.exec_action('gotoRoom' , 'r_entrance')
    wait_for_person(p)
    # 2. ask the person to go in front of the robot to get their details
    #p.exec_action("speak", "Please_move_one_metre_in_front_of_me_so_that_I_can_see_you_better.")
    #p.exec_action("speak", "Say,_hey_tiago_continue,_once_you_have_done_so.")

    #p.exec_action("listen", "continue") # TODO this waits that someone confirms the robot can continue
    #p.get_condition('isPersonDetected_guest1')
    #p.action_cmd('Recog' , "" , "start")


    p.exec_action('speak' , 'Hi_I_am_tiago,_welcome_to_the_party!')
    p.exec_action('speak' , 'Can_you_stand_there_and_look_at_my_eyes,_please?')
    while not p.get_condition("isPersonDetected"):
        time.sleep(1)
    #p.exec_action('findClosestPersonToTrack' , '')
    p.exec_action('personIdentification' , 'learn' )
    while not rospy.get_param('personSaved'): time.sleep(0.1)
    p.exec_action('saveGuestData' , 'setid_guest2')
    #p.exec_action('speak' , 'can_you_tell_your_name?')
    #p.exec_action('activateRasa' , 'guest_name')
    obtain_person_information(p , 'guest2' , 'name')
    obtain_person_information(p , 'guest2' , 'drink')
    p.exec_action('speak' , 'Please_follow_me_to_the_party_' + rospy.get_param('guest2/name'))
    p.exec_action('gotoRoom' , 'livingroom')#TODO set location of where to go 
    indtroduce_people('host' , 'guest2')
    indtroduce_people('guest1' , 'guest2')
    p.exec_action('gotoRoom' , 'r_couch')
    p.exec_action('speak' , 'Please_make_your_comfortable_on_the_couch_' + str(rospy.get_param('/guest2/name')))
    time.sleep(10)
    p.exec_action('speak' , 'Lets_have_some_fun!!!!')











    # implement rasa actions and calling name and drink 
    #p.exec_action('saveGuestData' , 'setname_guest1')
    #while not p.get_condition('isPersonDetected_guest1'):
    #    rospy.sleep(0.01)
    #p.exec_action('speak' , 'guest1_detected')
    #name = rospy.get_param('/guest1/name')
    #p.exec_action('speak', name)
    #identify_person(p)
    

    

    # 3. after reaching the car, the operator takes the bag back and thanks the robot.

    # 4. return to the arena
    p.exec_action("goto", "arenaEntrance")  # TODO


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    Receptionist(p)

    p.end()
