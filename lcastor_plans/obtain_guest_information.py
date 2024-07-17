import os
import sys
import rospy
import time
from std_msgs.msg import String
import random
from request_ollama import request_ollama
from AskConfirmation import AskConfirmation

from ollamamessages.msg import WhisperListening, OllamaResponse

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + "/scripts")
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import time
import pnp_cmd_ros
from pnp_cmd_ros import *
from confirm_information_simple import confirm_information_simple

OVERALL_TIMEOUT = 30.0
RESPONSE_TIMEOUT = 5.0
MAX_TRIES = 2


def obtain_guest_information(p, person, info, tries=0):
    """
    person: "guest1" or "guest2"
    info: "name" or "drink"
    """

    person = person.lower()
    info = info.lower()

    if info == "name":
        speech_text = "Please_tell_me_your_name?"
        default_info = random.choice(["max", "tom", "alex", "julie", "farah", "tammie"])
        final_text = "Thank_you_"
    elif info == "drink":
        speech_text = "Whats_your_favourite_drink?"
        default_info = random.choice(
            ["milk", "wine", "orange_juice", "hot_chocolate", "coffee"]
        )
        final_text = "Hopefully_we_can_serve_you_some_"
    else:
        rospy.logerr("obtain_person_info: invalid info type.")
        return

    topic = f"guest_{info}"

    success = False
    while not success and tries < MAX_TRIES:
        time.sleep(1)
        success, info_response = request_ollama(
            p, info, topic, speech_text, default_info
        )
        if not success:
            tries += 1
        if tries >= MAX_TRIES:
            info_response = error_setting_defaults(p, info, person, default_info)
            return

    # affirm_tries = 0
    affirm_success = False

    # case 1: ollama returns a false response (try again)
    affirm_success, affirm_response = AskConfirmation(
        p,
        speech_text=f"Is_your_{info}_{info_response}?",
        cannot_hear_text="please_try_saying,_correct_or_reject.",
    )

    if not affirm_success:
        rospy.loginfo(f"Struggling with affirm {person}/{info}, setting defaults")
        info_response = error_setting_defaults(p, info, person, default_info)
        return

    # case 2: info is rejected by user (try again)
    if affirm_response == "no":
        tries += 1
        info_response = obtain_guest_information(p, person, info, tries=tries)
        return

    # case 2: info and affirmed by user (success case)
    if affirm_response == "yes":
        time.sleep(2)
        rospy.loginfo(f"obtain_guest_info: saving guest data for {person}...")
        p.exec_action("saveGuestData", f"set{info}_" + person + "_" + info_response)
        get_info = rospy.get_param(f"/{person}/{info}")
        p.exec_action("speak", final_text + get_info)
        return

    error_setting_defaults(p, info, person, default_info)


def error_setting_defaults(p, info, person, default_info):
    rospy.loginfo(
        f"obtain_guest_information: failed to obtain {info} for {person}, using default"
    )
    p.exec_action("saveGuestData", f"set{info}_" + person + "_" + default_info)
    speech = (
        f"sorry,_im_struggling_to_catch_your_{info}._It_will_be_set_to_{default_info}."
    )
    p.exec_action("speak", speech)
    return default_info


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    obtain_guest_information(p, "guest1", "name")
    obtain_guest_information(p, "guest1", "drink")


    p.end()
