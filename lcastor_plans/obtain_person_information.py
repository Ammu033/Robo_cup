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
from confirm_information import confirm_information

def obtain_person_information(p, person, info):
    person = person.lower()
    info = info.lower()
    confirmed = False

    while not confirmed:

        if info == "name":
            
            p.exec_action('speak', "Can_you_please_tell_me_your_name?")
            p.exec_action('activateRasa', "guest_name")

        elif info == "drink":
            
            p.exec_action('speak', "Can_you_please_tell_me_your_favourite_drink?")
            p.exec_action('activateRasa', "guest_drink")

        # TODO add touchscreen input if too long
        while not p.get_condition("IsIntentDetected"):
            time.sleep(1)

        if info == "name":
            p.exec_action("saveGuestData", "setname_" + person)

        elif info == "drink":
            p.exec_action("saveGuestData", "setdrink_" + person)
        

        confirmed = confirm_information(p, person, info)


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    obtain_person_information(p, "guest1", "name")

    p.end()
