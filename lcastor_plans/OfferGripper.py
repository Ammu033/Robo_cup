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
import rospy
from std_msgs.msg import Bool


QUESTION_TIMEOUT = 3
def OfferGripper(p, msg):
    confirmed = False
    
    p.exec_action("armAction", "offer")
    p.exec_action("gripperAction", "open")

    n = 0
    while not confirmed:
        if n == 2:
            break
        p.exec_action("speak", msg)
        p.exec_action("activateRasa", "affirm_deny")
        try:
            confirmed = rospy.wait_for_message('/person_affirm_deny' , Bool, timeout=QUESTION_TIMEOUT).data
        except:
            confirmed = False

        n += 1
            
    
    p.exec_action("gripperAction", "close")
    p.exec_action("armAction", "home")


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    OfferGripper(p, "Please_confirm_when_you_have_placed_the_bag_in_my_hand.")

    p.end()
