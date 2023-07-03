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
from PersonFollowing import PersonFollowing
from OfferGripper import OfferGripper
def CarryMyLuggage(p):
    
    # 1. Wait for the door open
    while(not p.get_condition("IsDoorOpen")): continue

    # 2. Go to the living room
    p.exec_action("gotoRoom", "livingroom")

    # 3. Picking up the bag: The robot picks up the bag pointed at by the operator.
    pickUp_bag(p)

    # 4. Setting Navigation Mode to MAPPING.
    p.exec_action('setNavigationMode', 'MAP')
    
    # 5. Display "Have we arrived?" msg on the display
    p.action_cmd('getYesNoConfirmation', 'Have_we_arrived?', 'start')

    # 5. Following the operator: The robot should inform the operator when it is ready to follow them.
    PersonFollowing(p) 
    p.action_cmd('getYesNoConfirmation', '', 'stop')

    # 6. after reaching the car, the operator takes the bag back and thanks the robot.
    OfferGripper(p, "Please_confirm_you_took_the_bag_from_my_hand.")
    
    # 7. Go back to the initial position
    p.exec_action('goto', '0.0_0.0_0.0')
    
    # 8. Setting Navigation Mode to LOCALISATION.
    p.exec_action('setNavigationMode', 'LOC')
    

if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    CarryMyLuggage(p)

    p.end()

