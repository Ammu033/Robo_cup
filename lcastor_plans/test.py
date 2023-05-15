import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import pnp_cmd_ros
from pnp_cmd_ros import *

def test(p):

    p.exec_action('moveHead', '0.6_0')

    p.exec_action('moveHead', '-0.6_0')

    p.exec_action('moveHead', '0_1')

    p.exec_action('moveHead', '0_-1')

if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    test(p)

    p.end()
