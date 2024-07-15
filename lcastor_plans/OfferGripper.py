import os
import sys
from AskConfirmation import AskConfirmation

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + "/scripts")
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import time
import pnp_cmd_ros
from pnp_cmd_ros import *
import rospy
from std_msgs.msg import Bool


QUESTION_TIMEOUT = 3


def OfferGripper(p, msg, retry_msg):
    confirmed = False

    p.exec_action("armAction", "offer")
    p.exec_action("gripperAction", "open")

    n = 0
    while not confirmed:
        if n == 2:
            break
        p.exec_action("speak", msg)
        AskConfirmation(
            p,
            speech_text=msg,
            cannot_hear_text=retry_msg,
        )
        try:
            confirmed = rospy.wait_for_message(
                "/person_affirm_deny", Bool, timeout=QUESTION_TIMEOUT
            ).data
        except:
            confirmed = False

        n += 1

    p.exec_action("gripperAction", "close")
    p.exec_action("armAction", "home")


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    OfferGripper(
        p,
        "Please_confirm_when_you_have_placed_the_bag_in_my_hand.",
        "Please_confirm_more_loudly.",
    )

    p.end()
