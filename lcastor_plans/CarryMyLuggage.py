import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import pnp_cmd_ros
from pnp_cmd_ros import *
from pickUp_bag import pickUp_bag
from PersonFollowing import PersonFollowing
from OfferGripper import OfferGripper


def CarryMyLuggage(p):  
    
    p.exec_action('moveHead', '0.0_0.0')
    p.exec_action("armAction", "home")

    # 3. Picking up the bag: The robot picks up the bag pointed at by the operator.
    pickUp_bag(p)

    # 4. Setting Navigation Mode to MAPPING and move the Torso and Head to navigate.
    p.exec_action('moveHead', '0.0_-0.75')

    # 5. Following the operator: The robot should inform the operator when it is ready to follow them.
    # p.exec_action('setNavigationMode', 'MAP')    
    PersonFollowing(p) 

    # 6. after reaching the car, the operator takes the bag back and thanks the robot.
    p.exec_action("moveHead", "0.0_0.0")
    p.exec_action("speak", "Please_take_the_bag_from_my_hand.")
    OfferGripper(
        p,
        "Please_say_confirm_when_you_have_taken_the_luggage.",
        "please_say,_I_confirm,_more_loudly_if_it_has_been_picked.",
    )

    # 7. Go back to the initial position
    p.exec_action('speak', "Thank_you,I_will_now_go_back_to_the_starting_position.")
    p.exec_action('moveHead', '0.0_-0.75')
    p.exec_action('goto', '0.0_0.0_0.0')
    
    # 8. Resetting initial settings
    p.exec_action('moveHead', '0.0_0.0')
    # p.exec_action('setNavigationMode', 'LOC')
    

if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    CarryMyLuggage(p)

    p.end()

