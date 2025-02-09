import sys
import os

sys.path.insert(1, os.path.join(os.path.dirname(__file__), "..", "..", "..", "lcastor_plans"))

import rospy
import capabilities.contexts as contexts
# import capabilities.receptionist as receptionist
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from PersonFollowing import PersonFollowing
from ollamamessages.msg import WhisperTranscription, WhisperListening
from ollamamessages.srv import OllamaCall, OllamaCallResponse
from ollamamessages.msg import OllamaResponse
import tempfile
import random
import ollama
import time
import cv2

# rospy.init_node("gpsr_executor")
# ollama_out_pub = rospy.Publisher(
#     "/ollama_output",
#     String,
#     queue_size=1,
#     latch=True,
# )

# ollama_api_url = rospy.get_param("/gpsr/ollama_api_url", "192.168.69.253:11434")
ollama_api_url = rospy.get_param("/gpsr/ollama_api_url", "127.0.0.1:11434")
ollama_multimodal_model = rospy.get_param("/gpsr/ollama_multimodal_model", 'llava:7b')

def exception_handling(func):
    def wrapper(*args, **kwargs):
        try:
            print("[EXECUTION LOG] Executing action \"{}\"".format(func.__name__))
            print("[EXECUTION LOG] stdout from execution will follow:")
            func(*args, **kwargs)
            print("[EXECUTION LOG] Action \"{}\" executed successfully.".format(func.__name__))
        except Exception as e:
            print("[EXECUTION LOG] !! Exception \"{}\" occurred during execution of \"{}\"".format(e, func.__name__))
    return wrapper

def publish_what_im_doing(what_im_doing):
    print(what_im_doing)
    # ollama_out_pub.publish('Generated subtask, I am going to "%s"' % what_im_doing)

@contexts.context(["gpsr"])
@exception_handling
def goto_location(p, location_name):
    """Go to a named location name, such as 'side tables', 'bedroom' or 'cabinet' or 'bedside table'

    Args:
        location_name (str): The location name to go to, as a string
    """
    publish_what_im_doing("goto_location(location_name='%s')" % location_name)

    p.exec_action('gotoRoom', "r_" + location_name)

@contexts.context(["gpsr"])
@exception_handling
def go_back_to_me(p):
    """Go back to me"""
    publish_what_im_doing("go_back_to_me()")

    p.exec_action('gotoRoom' , 'r_inspectionpoint')

@contexts.context(["gpsr"])
@exception_handling
def grasp_object(p, object_name):
    """Grasps a given object, for example 'fruit' or 'bowl'

    Args:
        object_name (str): The name of the object to grasp
    """
    if object_name is None:
        object_name = "the object"
    publish_what_im_doing("grasp_object(object_name='%s')" % object_name)

    p.exec_action('speak', 'I_am_not_able_to_grasp_the_{}.'.format(object_name.replace(" ", "_")))
    p.action_cmd('speak', 'Can_you_place_it_in_my_hand?', 'start')
    p.exec_action("gripperAction", "open")
    p.action_cmd('speak', 'Can_you_place_it_in_my_hand?', 'stop')
    time.sleep(3)
    p.exec_action("gripperAction", "close")
    p.exec_action('speak', 'Thanks')

@contexts.context(["gpsr"])
@exception_handling
def offer_object(p):
    """Grasps the object currently being held. This means that `grasp_object()`,
    and go_back_to_me() must have been previously called."""
    publish_what_im_doing("offer_object()")

    p.exec_action('speak', 'Here_is_the_item_you_have_requested,_please_take_it_from_my_hand.')
    time.sleep(3)
    p.exec_action("gripperAction", "open")

@contexts.context(["gpsr"])
@exception_handling
def ask_for_person(p, person_name):
    """Ask for a person with a given name, for example 'Angel' or 'Morgan'.
    It can also be a descriptive action, e.g. 'person pointing to the right'. It can be a description not a name.
    You probably should have called `goto_location()` first.

    Args:
        person_name (str): The person's name to ask for
    """
    publish_what_im_doing("ask_for_person(person_name='%s')" % person_name)

    p.exec_action('speak', 'I_am_looking_for_{}'.format(person_name.replace(" ", "_")))
    p.exec_action('speak', 'Can_you,_{},_please_come_in_front_of_me?'.format(person_name.replace(" ", "_")))
    time.sleep(3)

@contexts.context(["gpsr"])
@exception_handling
def identify_people(p, what_to_identify):
    """Identifies and counts the number of people in a room doing a given action.
    For example what_to_identify could be 'standing persons' or 'pointing to the right'.
    This function should be called if 'pose' is in the prompt.

    Args:
        what_to_identify (str): Action to identify
    """
    publish_what_im_doing("identify_people(what_to_idenfify='%s')" % what_to_identify)
    engine_say(p, "I am identifying people who are %s" % what_to_identify)

    bridge = CvBridge()
    im = rospy.wait_for_message(
        "/xtion/rgb/image_raw", Image, timeout = 1
    )
    cv_image = bridge.imgmsg_to_cv2(im, "bgr8")
    with tempfile.TemporaryDirectory() as tf:
        im_path = os.path.join(tf, "identification_im.jpg")
        cv2.imwrite(im_path, cv_image)
        print(im_path, os.path.getsize(im_path))
        prompt = "You are the perception part of a mobile helper robot. Tell me about the '%s' about the people in this image. \
        You should only refer to this and not to anything else in the scene. You should reply as shortly as possible. \
        Do not refer to the image at all, pretend it is your eyes instead. \
        Do not say that you are a chatbot or an AI assistant. Do not mention 'image' or 'images'." % what_to_identify
        print(prompt)
        # prompt = "Describe the image"
        
        client = ollama.Client(host = "http://%s" % ollama_api_url)
        ollama_output = client.generate(
            model = ollama_multimodal_model, 
            prompt = prompt,
            images = [im_path], 
            keep_alive = "0m"
        )["response"]
    print(ollama_output)
    time.sleep(1)
    rospy.set_param("/gpsr/saved_information", ollama_output)

@contexts.context(["gpsr"])
@exception_handling
def identify_objects(p, what_to_idenfify):
    """Given something to look for, for example, the biggest food item or the smallest toy
    or the number of plates, do perception to indentify this. 'Tell me' tasks consist of an
    `identify_objects()` task followed by a `report_information()` task after the robot has
    moved back to its reporting position. You probably need to go to a location before calling this.

    Args:
        what_to_idenfify (str): Something to identify
    """
    publish_what_im_doing("identify_object(what_to_idenfify='%s')" % what_to_idenfify)
    engine_say(p, "I am identifying %s" % what_to_idenfify)

    p.exec_action('moveHead', '0.0_-0.8')
    
    bridge = CvBridge()
    im = rospy.wait_for_message(
        "/xtion/rgb/image_raw", Image, timeout = 1
    )
    cv_image = bridge.imgmsg_to_cv2(im, "bgr8")
    with tempfile.TemporaryDirectory() as tf:
        im_path = os.path.join(tf, "identification_im.jpg")
        cv2.imwrite(im_path, cv_image)
        print(im_path, os.path.getsize(im_path))
        prompt = "You are the perception part of a mobile helper robot. Tell me about the '%s' in this image. \
        You should only refer to this and not to anything else in the scene. You should reply as shortly as possible. \
        Do not refer to the image at all, pretend it is your eyes instead: for example, 'I can't see any toys'. \
        Do not say that you are a chatbot or an AI assistant. Do not mention 'image' or 'images'. Do not say sorry ever." % what_to_idenfify
        print(prompt)
        # prompt = "Describe the image"
        
        client = ollama.Client(host = "http://%s" % ollama_api_url)
        ollama_output = client.generate(
            model = ollama_multimodal_model, 
            prompt = prompt,
            images = [im_path], 
            keep_alive = "0m"
        )["response"]
    print(ollama_output)
    time.sleep(1)
    rospy.set_param("/gpsr/saved_information", ollama_output)

    p.exec_action('moveHead', '0.0_0.0')

@contexts.context(["gpsr"])
@exception_handling
def report_information(p):
    """Report back a previous identification task. This therefore means that `identify_objects()`
    must previously have been called. You must go back to you (the inspection point) before calling this function.
    """
    publish_what_im_doing("report_information()")

    time.sleep(1)
    report = rospy.get_param("/gpsr/saved_information", "I am not sure.")
    # p.exec_action('speak', 'I_have_identified_{}'.format(report.replace(" ", "_")))
    engine_say(p, report)

@contexts.context(["gpsr"])
@exception_handling
def salute(p):
    """Salute a person"""
    publish_what_im_doing("salute()")

    p.action_cmd("armAction", "wave", "start")
    p.exec_action('speak', 'Hello!')
    p.action_cmd("armAction", "wave", "stop")

@contexts.context(["gpsr"])
@exception_handling
def follow_person(p):
    """Follow the person directly in front of you. This means `ask_for_person()` must previously have been called."""
    publish_what_im_doing("following someone")

    PersonFollowing(p)

@contexts.context(["gpsr"])
def guide_person(p):
    """Ask a human to follow the robot. `goto_location()` and `ask_for_person()` must have previously been called, in that order. You must go to a location
    immediately after this. You should only call this function if the verb 'guide' or 'take' or 'escort' is in the prompt."""
    publish_what_im_doing("guide_person()")

    engine_say(p, "Please, follow me")

@contexts.context(["gpsr"])
@exception_handling
def cease_all_motor_functions(p):
    engine_say(p, "Like fire and powder, which as they kiss consume,")
    engine_say(p, "The sweetest honeys loathsome in his own deliciousness,")
    engine_say(p, "These violent delights have violent ends.")

@contexts.context(["gpsr"])
@exception_handling
def answer_quiz(p):
    """Answers a person's quiz. This should only be called if 'quiz' specifically is in the prompt.
    Moreover, `ask_for_person()` should have been the last function called.
    """
    listening_pub = rospy.Publisher("/stt/listening", WhisperListening, queue_size=1)
    intent_publisher = rospy.Publisher("/planner_intention", String, queue_size=1, latch = True)
    rospy.set_param("/stt/use_ollama", True)

    intent_publisher.publish("quiz")
    engine_say(p, "Hello, I am supposed to answer your quiz. Please ask me a question.")
    time.sleep(0.5)
    listening_pub.publish(listening=True)

    # this is not the correct way to do ollama stuff, therefore, this can only
    # be called once otherwise it'll just repeat the previous answer
    # TODO: fix this shit!
    try:
        info_output = rospy.wait_for_message(
            "ollama_output", String, timeout=30
        ).data
        engine_say(p, info_output)
    except:
        engine_say(p, "I'm sorry, even though I understood your question, I'm too stupid to know the answer.")
    
    # engine_say(p, "I understood your question was %s. Let me think about that for a second" % text)


@contexts.context(["gpsr"])
@exception_handling
def engine_say(p, to_say):
    publish_what_im_doing("engine_say('%s')" % to_say)
    p.exec_action('speak' , to_say.replace(" ", "_"))

    # p.exec_action('moveHead', '%d_0.0' % random.randint(-10, 10) / 10)

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
    time.sleep(5)
    identify_people(p, "number of people sitting down")
    # report_information(p)
    
    p.end()
