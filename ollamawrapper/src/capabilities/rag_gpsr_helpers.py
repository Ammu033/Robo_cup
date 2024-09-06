from difflib import SequenceMatcher
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
import tempfile
import ollama
import time
import cv2
import sys
import os

ollama_api_url = rospy.get_param("/gpsr/rag/ollama_api_url", "192.168.69.54:11434")
ollama_multimodal_model = rospy.get_param("/gpsr/rag/ollama_multimodal_model", 'llava:7b')

# could use the python logger module but idgaf
def llog(msg):
    print(msg)
    with open(os.path.join(os.path.dirname(__file__), "..", "..", "contexts", "gpsr_runner.log"), "a") as f:
        f.write(msg + "\n")

def exception_handling(func):
    def wrapper(*args, **kwargs):
        try:
            llog("[EXECUTION LOG] Executing action \"{}\"".format(func.__name__))
            llog("[EXECUTION LOG] stdout from execution will follow:")
            func(*args, **kwargs)
            llog("[EXECUTION LOG] Action \"{}\" executed successfully.".format(func.__name__))
        except Exception as e:
            llog("[EXECUTION LOG] !! Exception \"{}\" occurred during execution of \"{}\"".format(e, func.__name__))
    return wrapper

def similar(a, b):
    return SequenceMatcher(None, a, b).ratio()

def identify_objects_primitive(p, what_to_idenfify):
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
    rospy.set_param("/gpsr/saved_information", ollama_output.strip())

    p.exec_action('moveHead', '0.0_0.0')

def identify_people_primitive(p, what_to_identify):
    """Identifies and counts the number of people in a room doing a given action.
    For example what_to_identify could be 'standing persons' or 'pointing to the right'.
    This function should be called if 'pose' is in the prompt.

    Args:
        what_to_identify (str): Action to identify
    """

    bridge = CvBridge()
    im = rospy.wait_for_message(
        "/xtion/rgb/image_raw", Image, timeout = 3
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
    rospy.set_param("/gpsr/saved_information", ollama_output.strip())

if __name__ == "__main__":
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

    identify_people_primitive(p, input("Input what people to identify: "))

    p.end()
