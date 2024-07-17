import os
import sys
import rospy

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + "/scripts")
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

    p.exec_action("speak", "Hi,_my_name_is_tiago!")
    p.exec_action("speak", "Can_you_stand_there_and_look_at_my_eyes?")

    detected = False
    start_time = rospy.get_time()
    detected = p.get_condition("isPersonDetected")
    while not detected:
        if rospy.get_time() - start_time > 10.0:
            p.exec_action("speak", "Please_move_a_bit,_so_I_can_see_you_better.")
            start_time = rospy.get_time()

        detected = p.get_condition("isPersonDetected")
        time.sleep(1)

    p.action_cmd("personIdentification", "learn", "start")
    start_time = rospy.get_time()
    time.sleep(2)
    obtain_guest_information(p, guest, "name")
    obtain_guest_information(p, guest, "drink")

    saved = rospy.get_param("personSaved")
    while not saved:
        if rospy.get_time() - start_time > 30.0:
            break
        time.sleep(1)
        saved = rospy.get_param("personSaved")

    p.action_cmd("personIdentification", "learn", "stop")

    if not saved:
        rospy.set_param("LastSavedid", 15384)
    p.exec_action("saveGuestData", "setid_{}".format(guest))

    guest_name = rospy.get_param(f"/{guest}/name")
    p.exec_action("speak", f"Thank_you_{guest_name}").replace(" ", "_")

    # p.exec_action('gotoRoom' , 'r_center')
    if guest == "guest2":
        p.exec_action("speak", "Please_follow_me_to_the_couch.")
        p.exec_action("gotoRoom", "r_couch")  # TODO PUT BACK change to couch 2
    elif guest == "guest1":
        p.exec_action("speak", "Please_follow_me_to_the_couch.")
        p.exec_action("gotoRoom", "r_couch")  # TODO PUT BACK
    p.exec_action("armAction", "offer", "start")
    p.exec_action(
        "speak",
        "Please_be_seated_on_the_couch_"
        + str(rospy.get_param("/{}/name".format(guest))),
    )
    p.exec_action("armAction", "home")

    # p.exec_action('speak', 'Have_a_seat.')
    p.exec_action("saveGuestData", "setloc_host")
    p.exec_action("saveGuestData", "setloc_" + guest)
    p.exec_action("saveGuestData", "setheadangle_" + guest + "_" + str(0.0))

    # TODO: Introducing people
    p.exec_action("speak", "Please_stay_here_for_now")
    introduce_people(p, "host", guest)

    if guest == "guest2":
        introduce_people(p, "guest1", "guest2")


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    do_guest(p, "guest1")
    do_guest(p, "guest2")

    p.end()
