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
from do_guest import do_guest
import time
import pnp_cmd_ros
from pnp_cmd_ros import *

def Receptionist(p):
    
    rospy.set_param('personSaved' , 0)

    rospy.set_param('/host/name' , 'john')
    rospy.set_param('/host/drink' , 'milk')

    while not rospy.get_param('modelLoaded'): time.sleep(0.01)

    
    # p.exec_action('gotoRoom' , 'r_couch1') #TODO PUT BACK
    time.sleep(2)
    p.exec_action('moveHead', '-1_0.3')
    p.exec_action('speak' , 'Hello_John,_I_am_your_receptionist_for_this_party!')
    p.exec_action('speak', 'Please,_come_in_front_of_me.')
    p.exec_action('speak' , 'Can_you_stand_in_front_of_me_and_look_at_my_eyes,_please?')

    p.exec_action('moveHead', '0_0.3')
    wait_for_person(p)
    p.exec_action('saveGuestData' , 'setloc_host')

    p.exec_action('personIdentification' , 'learn' )
    start_time = rospy.get_time()
    saved = rospy.get_param('personSaved')
    while not saved: 
        if rospy.get_time() - start_time > 30.:
            break
        saved = rospy.get_param('personSaved')
        time.sleep(1)
    if not saved:
        rospy.set_param('LastSavedid' , 15384)

    p.exec_action('saveGuestData' , 'setid_host')
    

    rospy.set_param('personSaved' , 0)

    p.exec_action('speak' , 'Thank_you,_I_will_wait_at_the_entrance_for_guests!')


    # p.exec_action('gotoRoom' , 'r_receptionentrance')
    
    do_guest(p, "guest1")
    
    p.exec_action('speak' , 'I_will_now_go_to_the_entrance_and_wait_for_other_guests,_feel_at_home!!!')

    p.exec_action('gotoRoom' , 'r_receptionentrance')

    do_guest(p, "guest2")

    time.sleep(10)
    p.exec_action('speak' , 'Lets_have_some_fun!!!!')


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    Receptionist(p)

    p.end()
