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

def pickUp_bag(p):
    p.exec_action("lookAt", "front_person") # TODO
    # tell the operator to point at the bag
    p.exec_action("speak", "Please_point_at_the_bag_I_should_pick_up.")
    # tell the operator to 
    p.exec_action("detectGesture", "pointing") # TODO
    p.exec_action("detectBag", "") # TODO

    # grasp the bag
    p.exec_action("grasp", "pointedBag") # TODO

    # 
    # p.add_ER('grasp', 'GraspFailed', ask_push_sentence +
    #         '; waitFor_FreeRun; restart_action')
    

if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    pickUp_bag(p)

    p.end()
