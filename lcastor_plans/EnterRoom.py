import os
import sys
import time

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)
import pnp_cmd_ros
from pnp_cmd_ros import *
import rospy



def EnterRoom(p):  


    p.exec_action("speak", "Can_you_please_open_the_door_for_me_?")
    while(not p.get_condition("IsDoorOpen")): time.sleep(0.1)
    p.exec_action("speak", "Thank_you_for_opening_the_door")
    # print("Door is open")
    p.exec_action("navigateForward", "2")
    # print("Arrived")
    p.exec_action("speak", "I_ve_arrived_to_the_room")

if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    EnterRoom(p)

    p.end()

