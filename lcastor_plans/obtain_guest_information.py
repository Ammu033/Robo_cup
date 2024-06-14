import os
import sys
import rospy
import time
from std_msgs.msg import String, Bool

from ollamamessages.msg import WhisperListening, OllamaResponse

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + "/scripts")
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import time
import pnp_cmd_ros
from pnp_cmd_ros import *
from confirm_information_simple import confirm_information_simple

OVERALL_TIMEOUT = 59.0
RESPONSE_TIMEOUT = 20.0
MAX_TRIES = 2 
def request_info_from_ollama(p, info, publish_info, speech_text, default_info, tries=MAX_TRIES, timeout=OVERALL_TIMEOUT, response_timeout=RESPONSE_TIMEOUT, retry_after=RESPONSE_TIMEOUT):
    p.exec_action("speak", speech_text)

    listening_pub = rospy.Publisher("/stt/listening", WhisperListening, queue_size=1)

    planner_intent_pub = rospy.Publisher(
        "/planner_intention", String, queue_size=1, latch=True
    )
    
    planner_intent_pub.publish(publish_info)
    time.sleep(1)
    listening_pub.publish(listening=True)
    response = False
    check = True
    message_failed = False
    info_output = None

    tries = 0
    initial_start_time = rospy.get_time()
    start_time = initial_start_time

    while check:
        if rospy.get_time() - initial_start_time > timeout:
            p.exec_action(
                "speak",
                f"I_did_not_understand_you.I_will_set_your_{info}_to_{default_info}.",
            )
            check = False
            message_failed = True
            continue

        if (rospy.get_time() - start_time) < retry_after:
            try:
                response = rospy.wait_for_message(
                    "ollama_response", OllamaResponse, timeout=response_timeout
                )
            except Exception as e:
                rospy.logerr(e)
                continue
                
            # success case
            if response and response.success:
                response_intent = response.intent.lower()
                if response_intent == publish_info:
                    info_output = rospy.wait_for_message(
                        "ollama_output", String, timeout=response_timeout
                    ).data

                    #FIXME: sometimes it seems like reponse isn't captured if overlapping with speech?
                    # the overlapping might not be the issue, need to investigate some more

                    #TODO: manage spaced words? eg. 'iced tea'
                    # if spced words replace with '+'

                    info_output = info_output.replace(' ', '+')

                    check = False if info_output else True
                    continue

            elif response and not response.success:
                if tries > MAX_TRIES:
                    p.exec_action(
                        "speak",
                        f"I_did_not_understand_you._Your_{info}_is_{default_info}.",
                    )
                    check = False
                    message_failed = True
                    continue

                else:
                    p.exec_action("speak", "I_did_not_understand_you.")
                    p.exec_action("speak", speech_text)

                    start_time = rospy.get_time()
                    planner_intent_pub.publish(publish_info)
                    tries += 1
                    continue
        else:
            listening_pub.publish(listening=False)
            p.exec_action("speak", "Please_repeat_louder.")
            start_time = rospy.get_time()
            planner_intent_pub.publish(publish_info)
            time.sleep(1)
            listening_pub.publish(listening=True)
            continue

    if not info_output or message_failed:
        info_output = default_info

    success = not message_failed
    rospy.loginfo(f'info captured: {info_output}')
    rospy.loginfo(f'success status: {success}')
    
    return (success, info_output)

def obtain_guest_information(p, person, info):
    """
    person: "guest1" or "guest2"
    info: "name" or "drink"
    """

    person = person.lower()
    info = info.lower()

    if info == "name":
        speech_text = "Please_tell_me_your_name?"
        default_info = "Max"
        final_text = "Thank_you_"
    elif info == "drink":
        speech_text = "Please_tell_me_your_favourite_drink?"
        default_info = "Milk"
        final_text = "Hopefully_we_can_serve_you_some_"
    else:
        rospy.logerr("obtain_person_info: invalid info type.")
        return

    topic = f"guest_{info}"
    success, info_response = request_info_from_ollama(p, info, topic, speech_text, default_info)

    if success:        
        affirm_topic = "affirm_deny"
        affirm_info = "affirm_deny"
        speech_text = f"Did_you_say_{info_response}?"
        default_info = "Yes"
        affirm_success, affirm_response = request_info_from_ollama(p, affirm_info, affirm_topic, speech_text, default_info)
   
        if affirm_success and affirm_response == "yes":
            time.sleep(2)
            rospy.loginfo(f"obtain_guest_info: saving guest data for {person}...")
            p.exec_action(
                "saveGuestDataOllama", f"set{info}_" + person + "_" + info_response 
            )

            get_info = rospy.get_param(f"/{person}/{info}")
            p.exec_action("speak", final_text+get_info)
        else:
            rospy.loginfo("Couldn't obtain guest info stuff, ie in the else space DEBUG")
        return 

    time.sleep(2)
    rospy.loginfo(f"obtain_person: using an incase value for {person} {info}...")
    p.exec_action(
        "saveGuestDataOllama", f"set{info}_" + person + "_" + default_info 
    )
    p.exec_action(
        "speak", f"Thank_you._{default_info}_was_stored_as_your_{info}"
    )
   
if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    obtain_guest_information(p, "guest1", "name")

    p.end()
