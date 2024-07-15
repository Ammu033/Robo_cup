import rospy
import capabilities.contexts as contexts
# import capabilities.receptionist as receptionist
from std_msgs.msg import String
import sys
import os

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import time
import pnp_cmd_ros
from pnp_cmd_ros import *

ollama_out_pub = rospy.Publisher(
    "/ollama_output",
    String,
    queue_size=1,
    latch=True,
)

def publish_what_im_doing(what_im_doing):
    print(what_im_doing)
    # ollama_out_pub.publish('Generated subtask, I am going to "%s"' % what_im_doing)

@contexts.context(["gpsr"])
def goto_location(location_name):
    """Go to a named location name, such as 'side tables', 'bedroom' or 'cabinet' or 'bedside table'

    Args:
        location_name (str): The location name to go to, as a string
    """
    publish_what_im_doing("goto_location(location_name='%s')" % location_name)

    p = PNPCmd()
    p.begin()
    p.exec_action('gotoRoom', "r_" + location_name)
    p.end()

@contexts.context(["gpsr"])
def go_back_to_me():
    """Go back to me"""
    publish_what_im_doing("go_back_to_me()")

    p = PNPCmd()
    p.begin()
    p.exec_action('gotoRoom' , 'home')
    p.end()

@contexts.context(["gpsr"])
def grasp_object(object_name):
    """Grasps a given object, for example 'fruit' or 'bowl'

    Args:
        object_name (str): The name of the object to grasp
    """
    publish_what_im_doing("grasp_object(object_name='%s')" % object_name)

@contexts.context(["gpsr"])
def offer_object():
    """Grasps the onject currently being held. This means that `grasp_object()
    must have previously been called"""
    publish_what_im_doing("offer_object()")

@contexts.context(["gpsr"])
def ask_for_person(person_name):
    """Ask for a person with a given name, for example 'Angel' or 'Morgan'.
    It can also be a descriptive action, e.g. 'person pointing to the right'

    Args:
        person_name (str): The person's name to ask for
    """
    publish_what_im_doing("ask_for_person(person_name='%s')" % person_name)

# @contexts.context(["gpsr"])
# def identify_people(what_to_identify):
#     """Identifies and counts the number of people in a room doing a given action.
#     For example what_to_identify could be 'standing persons' or 'pointing to the right'

#     Args:
#         what_to_identify (str): Action to identify
#     """
#     publish_what_im_doing("identify_people(what_to_idenfify='%s')" % what_to_identify)

@contexts.context(["gpsr"])
def identify_objects(what_to_idenfify):
    """Given something to look for, for example, the biggest food item or the smallest toy
    or the number of plates, do perception to indentify this.

    Args:
        what_to_idenfify (str): Something to identify
    """
    publish_what_im_doing("identify_object(what_to_idenfify='%s')" % what_to_idenfify)

@contexts.context(["gpsr"])
def report_information():
    """Report back a previous identification task. This therefore means that `identify_objects()`
    must previously have been called.
    """
    publish_what_im_doing("report_information()")

@contexts.context(["gpsr"])
def salute():
    """Salute a person"""
    publish_what_im_doing("Salute")

@contexts.context(["gpsr"])
def follow_person():
    """Follow the person directly in front of you. This means `ask_for_person()` must previously have been called."""
    publish_what_im_doing("following someone")

@contexts.context(["gpsr"])
def done():
    publish_what_im_doing("done()")

if __name__ == "__main__":
    goto_location("table")