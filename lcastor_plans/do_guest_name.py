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

from obtain_person_name import obtain_person_name
from introduce_people import introduce_people
from wait_for_person import wait_for_person


def do_guest_name(p, guest):

    wait_for_person(p)

    p.exec_action("speak", "Hi,_my_name_is_tiago!")
    p.exec_action("speak", "Can_you_stand_there_and_look_at_my_eyes,_please?")

    obtain_person_name(p, guest, "name")
    obtain_person_name(p, guest, "drink")

    p.exec_action(
        "speak",
        "Thank_you_" + rospy.get_param(f"/{guest}/name".replace(" ", "_"),
    )
    p.exec_action("speak", "Enjoy_the_rest_of_the_day!")


if __name__ == "__main__":

    p = PNPCmd()
    p.begin()
    do_guest_name(p, "guest1")
    p.end()
