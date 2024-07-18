import rospy
import capabilities.contexts as contexts
# import capabilities.receptionist as receptionist
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
import tempfile
import random
import ollama
import time
import sys
import cv2
import os

# rospy.init_node("gpsr_executor")
# ollama_out_pub = rospy.Publisher(
#     "/ollama_output",
#     String,
#     queue_size=1,
#     latch=True,
# )

ollama_api_url = rospy.get_param("/gpsr/ollama_api_url", "192.168.69.253:11434")
# ollama_api_url = rospy.get_param("/gpsr/ollama_api_url", "127.0.0.1:11434")
ollama_multimodal_model = rospy.get_param("/gpsr/ollama_multimodal_model", 'llava:7b')

def publish_what_im_doing(what_im_doing):
    print(what_im_doing)
    # ollama_out_pub.publish('Generated subtask, I am going to "%s"' % what_im_doing)

@contexts.context(["gpsr"])
def goto_location(p, location_name):
    """Go to a named location name, such as 'side tables', 'bedroom' or 'cabinet' or 'bedside table'

    Args:
        location_name (str): The location name to go to, as a string
    """
    publish_what_im_doing("goto_location(location_name='%s')" % location_name)

    p.exec_action('gotoRoom', "r_" + location_name)

@contexts.context(["gpsr"])
def go_back_to_me(p):
    """Go back to me"""
    publish_what_im_doing("go_back_to_me()")

    p.exec_action('gotoRoom' , 'r_home')

@contexts.context(["gpsr"])
def grasp_object(p, object_name):
    """Grasps a given object, for example 'fruit' or 'bowl'

    Args:
        object_name (str): The name of the object to grasp
    """
    publish_what_im_doing("grasp_object(object_name='%s')" % object_name)

@contexts.context(["gpsr"])
def offer_object(p):
    """Grasps the onject currently being held. This means that `grasp_object()
    must have previously been called"""
    publish_what_im_doing("offer_object()")

@contexts.context(["gpsr"])
def ask_for_person(p, person_name):
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
def identify_objects(p, what_to_idenfify):
    """Given something to look for, for example, the biggest food item or the smallest toy
    or the number of plates, do perception to indentify this. 'Tell me' tasks consist of an
    `identify_objects()` task followed by a `report_information()` task after the robot has
    moved back to its reporting position. You probably need to go to a location before calling this.

    Args:
        what_to_idenfify (str): Something to identify
    """
    publish_what_im_doing("identify_object(what_to_idenfify='%s')" % what_to_idenfify)

    p.exec_action('moveHead', '0.0_-0.5')
    
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
        Do not say that you are a chatbot or an AI assistant." % what_to_idenfify
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
    rospy.set_param("/gpsr/saved_information", ollama_output)

    p.exec_action('moveHead', '0.0_0.0')

@contexts.context(["gpsr"])
def report_information(p):
    """Report back a previous identification task. This therefore means that `identify_objects()`
    must previously have been called. You must go back to you before calling this function.
    """
    publish_what_im_doing("report_information()")

@contexts.context(["gpsr"])
def salute(p):
    """Salute a person"""
    publish_what_im_doing("salute()")

@contexts.context(["gpsr"])
def follow_person(p):
    """Follow the person directly in front of you. This means `ask_for_person()` must previously have been called."""
    publish_what_im_doing("following someone")

@contexts.context(["gpsr"])
def guide_person(p):
    """Ask a human to follow the robot. `ask_for_person()` must have previously been called. You must go to a location
    immediately after this."""
    publish_what_im_doing("guide_person()")

@contexts.context(["gpsr"])
def cease_all_motor_functions(p):
    engine_say(p, "Like fire and powder, which as they kiss consume,")
    engine_say(p, "The sweetest honeys loathsome in his own deliciousness,")
    engine_say(p, "These violent delights have violent ends")

@contexts.context(["gpsr"])
def engine_say(p, to_say):
    publish_what_im_doing("engine_say(%s)" % to_say)
    # p.exec_action('speak' , to_say.replace(" ", "_"))

    p.exec_action('moveHead', '%d_0.3' % random.randint(-10, 10) / 10)

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

    identify_objects(p, "toys")
    
    p.end()
