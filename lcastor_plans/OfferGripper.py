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


def OfferGripper(p, msg):

    p.exec_action("armAction", "offerGripper")
    p.exec_action("gripperAction", "open")

    # p.exec_action("speak", "Please_confirm_when_you_have_placed_the_bag_in_my_hand.")
    p.exec_action("speak", msg)

    p.exec_action("activateRasa", "affirm")

    p.exec_action("listen", "")

    #TODO activate textinput if no speech answer
    p.add_ER("listen", "timeout_15", "")



if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    OfferGripper(p, "Please_confirm_when_you_have_placed_the_bag_in_my_hand.")

    p.end()
