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

default_names = ["Josie", "Tom"]
default_drinks = ["coke", "white wine"]

def identify_person(p):

    for _ in range(3):
        p.exec_action("personIdentification", "")
        if p.get_condition("PersonID_identified"):
            break
    
    if p.get_condition("PersonID_identified"):
        p.exec_action("speak", "Thank_you,I_hope_you_don't_mind_that_I_have_scanned_you!")
    else:
        p.exec_action("speak", "I_am_sorry,I_haven't_been_able_to_identify_you_properly")

    # TODO handle when name is not properly recognised
    # TODO check: is there a pre-defined set of names?
    p.exec_action("speak", "What_is_your_name?")
    p.exec_action("listen", "name")

    # TODO check: is there a pre-defined set of drinks?
    p.exec_action("speak", "What_is_your_favourite_drink?")
    p.exec_action("listen", "drink")




if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    identify_person(p)

    p.end()

