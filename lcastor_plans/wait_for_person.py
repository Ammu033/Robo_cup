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

def wait_for_person(p):

    # TODO this should be calibrated to make sure that a person is identifiable from the camera view
    p.exec_action('moveHead', '0_0')

    #  
    p.action_cmd("detectPerson", "", "start")


    while p.get_condition("PersonDetection_false"):
        time.sleep(3)

    p.action_cmd("detectPerson", "", "stop")




if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    wait_for_person(p)

    p.end()
