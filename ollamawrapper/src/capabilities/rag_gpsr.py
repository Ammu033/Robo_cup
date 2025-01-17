import sys
import os

sys.path.insert(1, os.path.join(os.path.dirname(__file__), "..", "..", "..", "lcastor_plans"))
sys.path.insert(2, os.path.join(os.path.dirname(__file__), "..", "..", "..", "lcastor_actions"))
import gotoRoom

import rospy
import rag_gpsr_helpers
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from PersonFollowing import PersonFollowing
from ollamamessages.msg import WhisperTranscription, WhisperListening
from ollamamessages.srv import OllamaCall, OllamaCallResponse
from ollamamessages.msg import OllamaResponse
import rag_gpsr_helpers
import hashlib
import tempfile
import random
import ollama
import math
import time
import cv2

ollama_api_url = rospy.get_param("/gpsr/ollama_api_url", "127.0.0.1:11434")
ollama_multimodal_model = rospy.get_param("/gpsr/ollama_multimodal_model", 'llava:7b')


def get_date(*args, **kwargs):
    """Returns the current date. May only be called inside an `engine_say()` function

    Returns:
        str: The current date
    """
    return "Tuesday"

def get_team_country(*args, **kwargs):
    """Returns the team name and country. May only be called inside an `engine_say()` function.

    Returns:
        str: The team name and country.
    """
    return "My team is called LCASTOR and we are from the United Kingdom"

@rag_gpsr_helpers.exception_handling
def goto_location(p, location_name):
    """Go to a named location name, such as 'side tables', 'bedroom' or 'cabinet' or 'bedside table'

    Args:
        location_name (str): The location name to go to, as a string
    """

    locations_dict = gotoRoom.ROOM_DICT[rospy.get_param("/arena")]
    # even in robocup's own documentation the room and location names are talked about
    # inconsistanty. so get the closest location name from the gotoRoom action
    theClosest = (None, 0)
    for a_name in locations_dict.keys():
        similarity = rag_gpsr_helpers.similar(location_name, a_name)
        if similarity > theClosest[1]:
            theClosest = (a_name, similarity)

    print("By '%s' I am assuming you mean '%s' (%.2f)" % (location_name, theClosest[0], theClosest[1]))
    location_name = theClosest[0]
    engine_say(p, "Going to the %s" % location_name)

    p.exec_action('gotoRoom', "r_" + location_name)

@rag_gpsr_helpers.exception_handling
def go_back_to_me(p):
    """Go back to me"""

    p.exec_action('gotoRoom' , 'r_inspectionpoint')

@rag_gpsr_helpers.exception_handling
def grasp_object(p, object_location, object_name):
    """Grasps a given object, for example 'fruit' or 'bowl' at a given location, e.g. 'kitchen table'

    Args:
        object_location (str): The location of the object to grasp
        object_name (str): The name of the object to grasp
    """
    if object_name is None:
        object_name = "the object"

    goto_location(p, location_name=object_location)
    # p.exec_action('speak', 'I_am_not_able_to_grasp_the_{}.'.format(object_name.replace(" ", "_")))
    # p.action_cmd('speak', 'Can_you_place_it_in_my_hand?', 'start')
    p.exec_action("gripperAction", "open")
    # p.action_cmd('speak', 'Can_you_place_it_in_my_hand?', 'stop')
    time.sleep(3)
    p.exec_action("gripperAction", "close")
    # p.exec_action('speak', 'Thanks')
    
@rag_gpsr_helpers.exception_handling
def offer_object_to_me(p):
    """Give the previously grasped object to me. `grasp_object()` must previously have been called"""
    go_back_to_me(p)

    engine_say(p, "Can you please take the object in my gripper?")
    time.sleep(2)
    p.exec_action("gripperAction", "open")
    time.sleep(2)
    p.exec_action("gripperAction", "close")
    engine_say(p, "Thank you.")

@rag_gpsr_helpers.exception_handling
def offer_object_to_person(p, person_location, person_name, object_name=""):
    """Offer a previously grasped object to a specific named person at a named location. You must
    be grasping an object, so `grasp_object()` must have previously been called. You should not
    use this function if 'me' is in the prompt, in that case, use `offer_object()` instead.

    Args:
        person_location (str): The location of the person, e.g. 'kitchen'
        person_name (str): The name of the person to offer the object to, e.g. 'Jesse'
    """
    ask_for_person(p, person_location, person_name)

    engine_say(p, "Can you please take the object in my gripper?")
    time.sleep(2)
    p.exec_action("gripperAction", "open")
    time.sleep(2)
    p.exec_action("gripperAction", "close")
    engine_say(p, "Thank you.")

@rag_gpsr_helpers.exception_handling
def place_object_on_location(p, location_name):
    """Place a previously grasped object on top of a named location. You must be grasping an object,
    `grasp_object()` must have previously been called.

    Args:
        location_name (str): The name of the location to put the object.
    """
    goto_location(p, location_name=location_name)
    # p.exec_action('speak', 'I_am_not_able_put_the_object_on_the_{}.'.format(location_name.replace(" ", "_")))
    # p.action_cmd('speak', 'Can_you_put_the_object_in_my_gripper_on_there_please?', 'start')
    p.exec_action("gripperAction", "open")
    time.sleep(3)
    p.exec_action("gripperAction", "close")
    # p.exec_action('speak', 'Thanks')

@rag_gpsr_helpers.exception_handling
def ask_for_person(p, person_location, person_name):
    """Ask for a person at a given location, with a given name, for example 'Angel' or 'Morgan'.
    The name can also be a descriptive action, e.g. 'person pointing to the right'. It can be a description not a name.

    Args:
        person_location (str): The person's location, e.g. 'kitchen'
        person_name (str): The person's name to ask for
    """
    goto_location(p, person_location)
    engine_say(p, "Person who is %s, please stand in front of me" % person_name)
    time.sleep(3)

@rag_gpsr_helpers.exception_handling
def identify_people(p, people_location, what_to_identify):
    """Identifies and counts the number of people in a given room doing a given action.
    For example what_to_identify could be 'standing persons' or 'pointing to the right'.
    This function should be called if 'pose' is in the prompt.

    Args:
        people_location (str): The location where people are
        what_to_identify (str): Action to identify
    """
    goto_location(p, people_location)
    rag_gpsr_helpers.identify_people_primitive(p, what_to_identify)

@rag_gpsr_helpers.exception_handling
def identify_objects(p, object_location, what_to_idenfify):
    """Given something to look for, for example, the biggest food item or the smallest toy
    or the number of plates, do perception to indentify this. 'Tell me' tasks consist of an
    `identify_objects()` task followed by a `report_information()` task after the robot has
    moved back to its reporting position.

    Args:
        object_location (str): Location to look identify an object, e.g. 'kitchen table'
        what_to_idenfify (str): Something to identify
    """
    goto_location(p, object_location)
    rag_gpsr_helpers.identify_objects_primitive(p, what_to_idenfify)

@rag_gpsr_helpers.exception_handling
def report_information(p):
    """Report back a previous identification task. This therefore means that `identify_objects()`
    must previously have been called. You must go back to you (the inspection point) before calling this function.
    """
    go_back_to_me(p)
    time.sleep(1)
    report = rospy.get_param("/gpsr/saved_information", "I am not sure.")
    # p.exec_action('speak', 'I_have_identified_{}'.format(report.replace(" ", "_")))
    engine_say(p, report)

@rag_gpsr_helpers.exception_handling
def salute(p):
    """Salute a person"""
    pass

@rag_gpsr_helpers.exception_handling
def greet_person(p, person_location, person_name):
    """Greet a person with a given name at a given location. The value for person_name
    can also be a description, such as 'the person wearing a gray jacket'.

    Args:
        person_location (str): A named location to find the person
        person_name (str): The name or a description of the person to greet
    """
    ask_for_person(p, person_location, person_name)
    engine_say(p, "Hello, %s" % person_name)

@rag_gpsr_helpers.exception_handling
def follow_person(p, person_name, from_location, to_location=""):
    """Follow a named person, or a description of a person, e.g. 'Alice', or 'person with their hand up'
    From a given location to a given location. The to parameter is optional.

    Args:
        person_name (str): Name of a person or a description of a person
        from_location (str): The name of the location from which to follow them
        to_location (str): (Optional) The name of the location to which to follow them
    """
    ask_for_person(p, from_location, person_name)
    engine_say(p, "I am following you now")
    # PersonFollowing(p)

@rag_gpsr_helpers.exception_handling
def guide_person(p, person_name, to_location, from_location=''):
    """  
    Ask a human to follow the robot. `goto_location()`  must have previously been called.
    You should only call this function if the verb 'guide' or 'take' or 'escort' is in the prompt.
    
    Args:
        to_location (str): Location guide the human to, e.g. 'kitchen table'
        person_name (str): The name of the person to guide, e.g. 'sally'
        from_location (str): (Optional) The name of the location to start
    """
    if from_location != '':
        ask_for_person(p, from_location, person_name)

    engine_say(p, "%s, please follow me to the %s" % (person_name, to_location))
    goto_location(p, to_location)

@rag_gpsr_helpers.exception_handling
def cease_all_motor_functions(p):
    engine_say(p, "Like fire and powder, which as they kiss consume,")
    engine_say(p, "The sweetest honeys loathsome in his own deliciousness,")
    engine_say(p, "These violent delights have violent ends.")

@rag_gpsr_helpers.exception_handling
def answer_quiz(p):
    """Answers a person's quiz or question. This should only be called if 'quiz' or 'question' specifically is in the prompt.
    Moreover, `ask_for_person()` should have been the last function called.
    """
    engine_say(p, "Placeholder")

    # listening_pub = rospy.Publisher("/stt/listening", WhisperListening, queue_size=1)
    # intent_publisher = rospy.Publisher("/planner_intention", String, queue_size=1, latch = True)
    # rospy.set_param("/stt/use_ollama", True)

    # intent_publisher.publish("quiz")
    # engine_say(p, "Hello, I am supposed to answer your quiz. Please ask me a question.")
    # time.sleep(0.5)
    # listening_pub.publish(listening=True)

    # # this is not the correct way to do ollama stuff, therefore, this can only
    # # be called once otherwise it'll just repeat the previous answer
    # # TODO: fix this shit!
    # try:
    #     info_output = rospy.wait_for_message(
    #         "ollama_output", String, timeout=30
    #     ).data
    #     engine_say(p, info_output)
    # except:
    #     engine_say(p, "I'm sorry, even though I understood your question, I'm too stupid to know the answer.")
    
    # # engine_say(p, "I understood your question was %s. Let me think about that for a second" % text)

@rag_gpsr_helpers.exception_handling
def engine_say(p, to_say):
    # p.exec_action('speak' , to_say.replace(" ", "_"))

    with open(os.path.join(os.path.dirname(__file__), "..", "..", "contexts", "speech.log"), "a") as f:
        f.write(to_say.replace(" ", "_") + "\n")

def say_task_impossible(p, reason):
    """
    This function is called when a requested task is impossible, for example when it is requested to go to a room
    that isn't in the scene, or to pick up an item that isn't on the given location.

    Args:
        reason (str): The reason why the task cannot be completed, for example "There are no bananas on the coffee table" when it is requested to bring a banana from the coffee when there are no bananas on the coffee table.
    """
    engine_say(p, "I cannot do this requested task because " + reason)

if __name__ == "__main__":
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
    import rospy
   
    p = PNPCmd()
    p.begin()

    # answer_quiz(p)
    # identify_objects(p, what_to_idenfify="number of foods")
    # identify_people(p, "number of people sitting down")
    # report_information(p)

    goto_location(p, input("Where would you like to go? "))

    # identify_objects(p, object_location='dinner table', what_to_idenfify='the lightest object')
    # go_back_to_me(p)
    # report_information(p)

    # engine_say(p, "Placeholder")
    # grasp_object(p, "kitchen counter", "bowl")
    
    p.end()
