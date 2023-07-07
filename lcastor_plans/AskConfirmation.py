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
from std_msgs.msg import Bool
import rospy


def AskConfirmation(p):
    confirmed = False    

    p.exec_action('activateRasa', "affirm_deny")
    
    start_time = rospy.get_time()
    detected = p.get_condition("IsIntentDetected")
    while not detected:
        if rospy.get_time() - start_time > 10.:
            p.exec_action('speak' , 'Please_repeat_louder,_I_did_not_understand_you.')
            p.exec_action('activateRasa', "affirm_deny")
            start_time = rospy.get_time()

        detected = p.get_condition("IsIntentDetected")
        time.sleep(1)

    try:
        confirmed = rospy.wait_for_message('/person_affirm_deny', Bool, timeout=5.)
    except Exception as e:
        confirmed = False    

    return confirmed







if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    confirm_information(p, "guest1", "name")

    p.end()
