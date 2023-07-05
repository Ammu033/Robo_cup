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

def confirm_information(p, person, info):

    confirmed = False

    data = rospy.get_param("/{}/{}".format(person, info))
    data = data.replace(" ", "_")

    p.exec_action("speak", "I_understood_your_{}_is_{}".format(info, data))
    p.exec_action("speak", "Please,_confirm_whether_that_is_correct.")


    p.exec_action('activateRasa', "affirm_deny")
    confirmed = rospy.wait_for_message('/person_affirm_deny' , Bool)
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
