import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import time
import pnp_cmd_ros
from pnp_cmd_ros import *
from wait_for_person import wait_for_person

def look_for_person(p , person):
    person_x = rospy.get_param(person + '/x')
    person_y = rospy.get_param(person  + '/y')
    person_w = rospy.get_param(person  + '/w')
    p.exec_action('goto' , str(person_x) + "_" + str(person_y) + "_"  + str(person_w))
    person_h_angle = float(rospy.get_param(person + '/head_angle'))
    p.exec_action('moveHead' , str(person_h_angle) + "_0.0")
    person_found = p.get_condition('isPersonDetected_'+ person)
    counter = 0
    if not person_found:
        while not person_found:
            counter = counter + 1
            start_angle = -1.3
            increament = 0.3
            while not p.get_condition("isPersonDetected_" + person):
                start_angle = start_angle + increament
                if start_angle >= 1.3 :
                    #p.exec_action('speak' , 'I_cannot_find_you,_but_please_stay_whereever_you_are_and_look_towards_me')
                    #p.exec_action('goto' , str(person_x) + str(person_y) + str(person_w +( 0.57 * counter)))# TODDO
                    person_found  = True
                    p.exec_action('saveGuestData' , 'setloc_' + person)
                    p.exec_action('saveGuestData' , 'setheadangle_' +person+'_'  + str(start_angle))
                    break
                p.exec_action('moveHead' , str(start_angle) + "_0.0")
            if not start_angle >= 1.3:
                person_found = True
                p.exec_action('saveGuestData' , 'setloc_' + person)
                p.exec_action('saveGuestData' , 'setheadangle_' +person+'_'  + '0.0')
            if counter > 0:
                break
    return person_found










if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    look_for_person(p , "host")

    p.end()
