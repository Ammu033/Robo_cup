import os
import sys
import rospy

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import time
import pnp_cmd_ros
from pnp_cmd_ros import *
import time
import pnp_cmd_ros
from pnp_cmd_ros import *

def OfferTest(p):

    p.exec_action('armAction', 'offer', 'start')
    p.exec_action('speak' , 'Please_be_seated')
    p.exec_action('armAction', 'home')
    p.exec_action('speak' , 'Lets_have_some_fun!!!!')


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    OfferTest(p)

    p.end()
