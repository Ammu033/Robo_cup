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
from pickUp_bag import pickUp_bag
from PersonFollowing import PersonFollowing
from OfferGripper import OfferGripper
def Inspection(p):
    
    # 0. Set Head position
    p.exec_action("moveHead", "0.0_-0.75")
    p.exec_action("moveTorso", "0.20")

    # 1. Wait for the door open
    p.exec_action("speak", "Can_you_please_open_the_door_for_me_?")
    while(not p.get_condition("IsDoorOpen")): time.sleep(0.1)
    p.exec_action("speak", "Thank_you_for_opening_the_door")

    # 2. Go to the living room
    p.exec_action("gotoRoom", "r_inspectionpoint")
    p.exec_action("speak", "Ready_for_the_inspection!")

if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    Inspection(p)

    p.end()

