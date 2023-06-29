import os
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

SPEAK_TIMEOUT = 5 # [s]
def people_id(p):
    #p.action_cmd('Recog' , "" , "start")
    p.exec_action('speak' , 'Can_you_stand_in_front_of_me,_please?')

if __name__ == "__main__":
    p = PNPCmd()
    p.begin()
    people_id(p)
    p.end()