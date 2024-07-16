import os
import sys
from request_ollama import request_ollama

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import time
import pnp_cmd_ros
from pnp_cmd_ros import *
from std_msgs.msg import Bool
import rospy

OVERALL_TIMEOUT = 30.0
RESPONSE_TIMEOUT = 5.0
MAX_TRIES = 2

def AskConfirmation(
        p,
        speech_text=f"Did_I_grab_it?",
        cannot_hear_text = "please_say,_yes_you_can_grab_it,_if_that_is_correct.",
        affirm_tries = 0,
        max_tries = MAX_TRIES,
    ):
    topic = "affirm_deny"
    info = "affirm_deny"
    default_info = "no"
    success = False
    affirm_tries = 0

    while not success:
        success, response = request_ollama(p, info, topic, speech_text, default_info, cannot_hear_text)

        if not success:
            rospy.loginfo(f"AskConfirmation: Did not recieve an affirm response")
            tries += 1

        if affirm_tries >= max_tries:
            rospy.loginfo(f"AskConfirmation: Too many tries")
            return (False, None)

    rospy.loginfo(f"AskConfirmation: Recieved response")
    return (success, response)






if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    AskConfirmation(p, "Did_I_grab_the_bag", "please_say,_yes_you_can_grab_it,_if_that_is_correct.")

    p.end()
