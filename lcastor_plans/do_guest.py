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

from obtain_guest_information import obtain_guest_information
from introduce_people import introduce_people
from wait_for_person import wait_for_person

def do_guest(p, guest):

    wait_for_person(p)

    p.exec_action('speak' , 'Hi,_my_name_is_tiago!')
    p.exec_action('speak' , 'Can_you_stand_there_and_look_at_my_eyes,_please?')
    ##
    detected = False
    start_time = rospy.get_time()
    detected = p.get_condition("isPersonDetected")
    while not detected:
       if rospy.get_time() - start_time > 10.:
           p.exec_action('speak' , 'Please_move_a_bit,_so_I_can_see_you_better.')
           start_time = rospy.get_time()#

       detected = p.get_condition("isPersonDetected")
       time.sleep(1)

    p.action_cmd('personIdentification' , 'learn','start')
    start_time = rospy.get_time()
    
    obtain_guest_information(p , guest , 'name')
    obtain_guest_information(p , guest , 'drink')

    saved = rospy.get_param('personSaved')
    while not saved: 
       if rospy.get_time() - start_time > 30.:
           break
       time.sleep(1)
       saved = rospy.get_param('personSaved')

    p.action_cmd('personIdentification' , 'learn', 'stop')

    if not saved:
       rospy.set_param('LastSavedid' , 15384)
    p.exec_action('saveGuestData' , 'setid_{}'.format(guest))

    p.exec_action('speak', 'Thank_you_' + rospy.get_param('/{}/name'.format(guest)).replace(" ", "_"))
    p.exec_action('gotoRoom' , 'r_couch1')
    #if guest == "guest2":
    #    p.exec_action('gotoRoom' , 'r_couch2') #TODO PUT BACK 
    #elif guest == "guest1":
    #    p.exec_action('gotoRoom' , 'r_couch1') #TODO PUT BACK 
    p.exec_action('saveGuestData' , 'setloc_host')
    p.exec_action('saveGuestData' , 'setloc_' + guest)
    p.exec_action('saveGuestData' , 'setheadangle_' +guest+'_'  + str(0.0))
    introduce_people(p, 'host' , guest)

    #if guest == "guest2":
    #    introduce_people(p, 'guest1' , 'guest2')

    #if guest == "guest2":
    #    p.exec_action('gotoRoom' , 'r_couch2') #TODO PUT BACK 
    #elif guest == "guest1":
    #    p.exec_action('gotoRoom' , 'r_couch1') #TODO PUT BACK 
    #p.action_cmd('armAction', 'offer', 'start')
    #p.exec_action('speak' , 'Please_be_seated_on_the_couch_' + str(rospy.get_param('/{}/name'.format(guest))))
    #time.sleep(1)
    #p.action_cmd('armAction', 'offer', 'stop')
    #p.exec_action('armAction', 'home')


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    do_guest(p, "guest1")
    do_guest(p, "guest2")

    p.end()
