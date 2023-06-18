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
from pickUp_bag import pickUp_bag
from follow_operator import follow_operator

def CarryMyLuggage(p):

    # 1. Picking up the bag: The robot picks up the bag pointed at by the operator.
    pickUp_bag(p) #TODO

    # 2. Following the operator: The robot should inform the operator when it is ready to follow them.
    follow_operator(p) #TODO

    # 3. after reaching the car, the operator takes the bag back and thanks the robot.


    # 4. return to the arena
    p.exec_action("goto", "arenaEntrance") # TODO

if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    CarryMyLuggage(p)

    p.end()

