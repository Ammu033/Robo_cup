import os
import sys
import rospy
import time
from std_msgs.msg import String
try:
    sys.path.insert(0, os.environ["PNP_HOME"] + "/scripts")
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import time
import pnp_cmd_ros
from pnp_cmd_ros import *
from confirm_information_simple import confirm_information_simple


def obtain_person_name(p, person, info):
    person = person.lower()
    info = info.lower()
    confirmed = False

    # TODO: while not confirmed starts up the second loop
    # while not confirmed:

    if info == "name":
        # TODO: the intention message needs to be sent *before* whisper is activated, otherwise, it might cause issues
        # currently, the sending of the intention message causes whisper to start at the same time, but I think this
        # should be separated into two actions, it sets the intention, then pauses, and then activates whisper
        p.exec_action("speak", "Can_you_please_tell_me_your_name?")
        time.sleep(1)
        rospy.loginfo("activating ollama...")
        p.exec_action("activateOllama", "guest_name")
        reponse_topic = "ollama_name"
    if info == "drink":
        p.exec_action("speak", "Can_you_please_tell_me_your_favourite_drink?")
        time.sleep(1)
        p.exec_action("activateOllama", "guest_drink")
        reponse_topic = "ollama_drink"

    start_time = rospy.get_time()

    # waiting on a response confirmation from the user
    rospy.loginfo("obtain_person_name: waiting for ollama intent detection...")

    condition = p.get_condition("IsOllamaIntentDetected")

    while not ( response := rospy.wait_for_message(reponse_topic, String, timeout=60).data ):
        # TODO: may need ollama wrapper to publish intent as soon as whisper starts

        # only attempt to ask again if 10 seconds have passed
        # maybe this should be some kind of timeout parameter
        if rospy.get_time() - start_time > 10.0:
            p.exec_action("speak", "Please_repeat_louder,_I_did_not_understand_you.")
            start_time = rospy.get_time()
            p.exec_action("activateOllama", "guest_" + info)
        time.sleep(1)

    time.sleep(2)
    if info == "name":
        rospy.loginfo(f"obtain_person_name: saving guest data for {person}...")
        p.exec_action("saveGuestDataOllama", "setname_" + person + '_' + response)

    if info == "drink":
        p.exec_action("saveGuestDataOllama", "setname_" + person + '_' + response)

    
    # p.exec_action
    # p.action_cmd('speak', f'Thank you_{rospy.get}', 'start')

        # TODO: (from above) confirmed starts up the second loop
        # confirmed = confirm_information_simple(p, person, info)


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    obtain_person_name(p, "guest1", "name")

    p.end()
