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
from introduce_people import introduce_people
import time
import pnp_cmd_ros
from pnp_cmd_ros import *

def Receptionist(p):
    
    rospy.set_param('personSaved' , 0)

    rospy.set_param('/host/name' , 'john')
    rospy.set_param('/host/drink' , 'milk')

    while not rospy.get_param('modelLoaded'): time.sleep(0.01)

    
    # p.exec_action('gotoRoom' , 'r_couch1') #TODO PUT BACK
    time.sleep(10)
    p.exec_action('moveHead', '-0.7_0.3')
    p.exec_action('speak' , 'Hello_John,_I_am_your_receptionist_for_this_party!')
    p.exec_action('speak', 'Please,_come_in_front_of_me.')
    p.exec_action('speak' , 'Can_you_stand_in_front_of_me_and_look_at_my_eyes,_please?')

    p.exec_action('moveHead', '0_0.3')
    wait_for_person(p)
    p.exec_action('saveGuestData' , 'setloc_host')

    p.exec_action('personIdentification' , 'learn' )
    while not rospy.get_param('personSaved'): time.sleep(0.1)
    p.exec_action('saveGuestData' , 'setid_host')

    rospy.set_param('personSaved' , 0)

    p.exec_action('speak' , 'Thank_you,_I_will_now_go_to_the_entrance_to_wait_for_guests!')


    # p.exec_action('gotoRoom' , 'r_receptionentrance')

    wait_for_person(p)


    p.exec_action('speak' , 'Hi,_I_am_tiago,_welcome_to_the_party!')
    p.exec_action('speak' , 'Can_you_stand_there_and_look_at_my_eyes,_please?')
    ##
    detected = False
    start_time = rospy.get_time()
    detected = p.get_condition("isPersonDetected")
    while not detected:
        if rospy.get_time() - start_time > 10.:
            p.exec_action('speak' , 'Please_move_a_bit,_so_I_can_see_you_better.')
            start_time = rospy.get_time()

        detected = p.get_condition("isPersonDetected")
        time.sleep(1)


    p.exec_action('personIdentification' , 'learn' )

    start_time = rospy.get_time()
    while not rospy.get_param('personSaved'): 
        if rospy.get_time() - start_time > 30.:
            break
        time.sleep(1)
    ##
    p.exec_action('saveGuestData' , 'setid_guest1')
    obtain_person_information(p , 'guest1' , 'name')
    obtain_person_information(p , 'guest1' , 'drink')
    p.exec_action('speak' , 'Please_follow_me_to_the_party,_' + rospy.get_param('guest1/name'))
    p.exec_action('gotoRoom' , 'r_couch2') #TODO PUT BACK 
    introduce_people(p, 'host' , 'guest1')
    p.exec_action('gotoRoom' , 'r_couch2') #TODO PUT BACK
    p.exec_action('speak' , 'Please_be_seated_on_the_couch_' + str(rospy.get_param('/guest1/name')))
    time.sleep(10)
    p.exec_action('speak' , 'I_will_now_go_to_the_entrance_and_wait_for_other_guests,_feel_at_home!!!')


    p.exec_action('gotoRoom' , 'r_receptionentrance')
    wait_for_person(p)

    
    rospy.set_param('personSaved' , 0)

    p.exec_action('speak' , 'Hi_I_am_tiago,_welcome_to_the_party!')
    p.exec_action('speak' , 'Can_you_stand_there_and_look_at_my_eyes,_please?')
    ##
    detected = False
    start_time = rospy.get_time()
    detected = p.get_condition("isPersonDetected")
    while not detected:
        if rospy.get_time() - start_time > 10.:
            p.exec_action('speak' , 'Please_move_a_bit,_so_I_can_see_you_better.')
            start_time = rospy.get_time()

        detected = p.get_condition("isPersonDetected")
        time.sleep(1)

    p.exec_action('personIdentification' , 'learn' )

    start_time = rospy.get_time()
    while not rospy.get_param('personSaved'): 
        if rospy.get_time() - start_time > 30.:
            break
        time.sleep(1)
    ##
    
    p.exec_action('saveGuestData' , 'setid_guest2')
    obtain_person_information(p , 'guest2' , 'name')
    obtain_person_information(p , 'guest2' , 'drink')
    p.exec_action('speak' , 'Please_follow_me_to_the_party_' + rospy.get_param('guest2/name'))
    p.exec_action('gotoRoom' , 'r_couch2')#TODO set location of where to go 
    introduce_people(p, 'host' , 'guest2')
    introduce_people(p, 'guest1' , 'guest2')
    p.exec_action('gotoRoom' , 'r_couch2')
    p.exec_action('speak' , 'Please_make_your_comfortable_on_the_couch_' + str(rospy.get_param('/guest2/name')))
    time.sleep(10)
    p.exec_action('speak' , 'Lets_have_some_fun!!!!')


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    Receptionist(p)

    p.end()
