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
from identify_person import identify_person


def Receptionist(p):

    # 1. The robot is in a predefined location near the entrance door
    wait_for_person(p)

    # 2. ask the person to go in front of the robot to get their details
    p.exec_action("speak", "Please_move_one_metre_in_front_of_me_so_that_I_can_see_you_better.")
    p.exec_action("speak", "Say,_hey_tiago_continue,_once_you_have_done_so.")

    p.exec_action("listen", "continue") # TODO this waits that someone confirms the robot can continue

    identify_person(p)
    
    

    

    # 3. after reaching the car, the operator takes the bag back and thanks the robot.

    # 4. return to the arena
    p.exec_action("goto", "arenaEntrance")  # TODO


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    Receptionist(p)

    p.end()
