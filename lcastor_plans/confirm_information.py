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

def confirm_information(p, person, info):

    confirmed = False

    data = rospy.get_param("/{}/{}".format(person, info))
    data = data.replace(" ", "_")

    p.exec_action("speak", "I_understood_your_{}_is_{}".format(info, data))
    p.exec_action("speak", "Please,_confirm_whether_that_is_correct.")

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
        confirmed = rospy.wait_for_message('/person_affirm_deny', Bool, timeout=10.).data
    except Exception  as e:
        confirmed = False    

    # if not affirm_deny_data:
    #     p.exec_action("speak", "Please,_press_yes_or_.")
    #     p.exec_action('getYesNoConfirmation', "Is_{}_your_{}".format(data, info))

    #     while True:
    #         if p.get_condition("IsYesConfirmed"):
    #             confirmed = True
    #         elif p.get_condition("IsNoConfirmed"):
    #             confirmed = False
    #         time.sleep(1)

    return confirmed







if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    confirm_information(p, "guest1", "name")

    p.end()
