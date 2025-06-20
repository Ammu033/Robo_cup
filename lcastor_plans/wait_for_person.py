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
    p.exec_action('moveTorso', '0.35')
    p.exec_action('moveHead', '0.0_0.2')
    

    # 
    n = 0
    while not p.get_condition("IsPersonInRange_1.2"):
        if n >= 4:
            break 
        time.sleep(3)
        n += 1
    






if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    wait_for_person(p)

    p.end()
