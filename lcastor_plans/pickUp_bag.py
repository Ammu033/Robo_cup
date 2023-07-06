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
from OfferGripper import OfferGripper

def pickUp_bag(p):

    # p.action_cmd("lookAtClosestPerson", "", "start")

    p.exec_action("speak", "Hello,I_will_help_you_carrying_your_bag_to_the_car.")
    p.exec_action("speak", "Sorry,_I_am_not_able_to_pick_up_the_bag.")
    p.exec_action("speak", "Can_you_please_put_it_in_my_hand?")
    
    OfferGripper(p, "Please_confirm_when_you_have_placed_the_bag_in_my_hand.")
    
    # p.action_cmd("lookAtClosestPerson", "", "stop")


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    pickUp_bag(p)

    p.end()