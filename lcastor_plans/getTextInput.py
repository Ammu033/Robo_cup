
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

def getTextInput(p):

    p.exec_action('speak', 'Hi_there,what_is_your_name?')

    p.exec_action('moveHead', '0_-0.4')

    p.exec_action('getTextInput', 'Please_insert_your_name')


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    getTextInput(p)

    p.end()
