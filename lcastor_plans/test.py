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

def test(p):

    if p.get_condition("JoyPriority_true"):
        p.exec_action('speak', 'I_can\'t_move._Please_give_priority_to_my_mobile_base!')

        while p.get_condition("JoyPriority_true"):
            time.sleep(1)

        p.exec_action('speak', 'Thank_you!')

    p.action_cmd('speak', 'Looking_left', 'start')
    p.exec_action('moveHead', '0.6_0')
    p.action_cmd('speak', 'Looking_left', 'stop')

    p.action_cmd('speak', 'Looking_right', 'start')
    p.exec_action('moveHead', '-0.6_0')
    p.action_cmd('speak', 'Looking_right', 'stop')

    p.action_cmd('speak', 'Looking_up', 'start')
    p.exec_action('moveHead', '0_1')
    p.action_cmd('speak', 'Looking_up', 'stop')

    p.action_cmd('speak', 'Looking_down', 'start')
    p.exec_action('moveHead', '0_-1')
    p.action_cmd('speak', 'Looking_down', 'stop')

if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    test(p)

    p.end()
