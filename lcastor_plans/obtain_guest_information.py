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




OVERALL_TIMEOUT = 120.0
RESPONSE_TIMEOUT = 60.0
RETRY_TIMEOUT=60.0
MAX_TRIES = 2
def request_info_from_ollama(p, info, publish_info, speech_text, default_info, tries=MAX_TRIES, timeout=OVERALL_TIMEOUT, response_timeout=RESPONSE_TIMEOUT, retry_after=RETRY_TIMEOUT):
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
            response = rospy.wait_for_message(
                "ollama_response", OllamaResponse, timeout=response_timeout
            )

            # success case
            if response and response.success:
                response_intent = response.intent.lower()
                if response_intent == publish_info:
                    info_output = rospy.wait_for_message(
                        "ollama_output", String, timeout=response_timeout
                    )
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
            p.exec_action("speak", "Please_repeat_louder.")
            start_time = rospy.get_time()
            planner_intent_pub.publish(publish_info)
            time.sleep(1)
            continue

    if not info_output or message_failed:
        info_output = default_info
    
    success = not message_failed
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
    elif info == "drink":
        speech_text = "Please_tell_me_your_favourite_drink?"
        default_info = "Milk"
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
    
        if affirm_success and affirm_response == "Yes":
        # TODO: Storing the variable of interest
            time.sleep(2)
            rospy.loginfo(f"obtain_person: saving guest data for {person}...")
            p.exec_action(
                "saveGuestDataOllama", f"set{info}_" + person + "_" + info_response 
            )
        if get_name := rospy.get_param(f"/{person}/name"):
            p.action_cmd("speak", f"Thank_you_{get_name}", "start")
        return 

    time.sleep(2)
    rospy.loginfo(f"obtain_person: using an incase value for {person} {info}...")
    p.exec_action(
        "saveGuestDataOllama", f"set{info}_" + person + "_" + default_info 
    )
    p.action_cmd(
        "speak", f"Thank_you._{default_info}_was_stored_as_your_{info}", "start"
    )

    # publish_info = f"guest_{info}"
    # p.exec_action("speak", speech_text)
    # planner_intent_pub.publish(publish_info)
    # time.sleep(1)
    # listening_pub.publish(listening=True)
    #
    # response = False
    # check = True
    # message_failed = False
    # MAX_TRIES = 2
    # overall_timeout = 120.0
    # ollama_timeout = 10.0
    # ollama_response_timeout = 5.0
    # tries = 0
    #
    # initial_start_time = rospy.get_time()
    # start_time = initial_start_time
    #
    # while check:
    #     if rospy.get_time() - initial_start_time > overall_timeout:
    #         p.exec_action(
    #             "speak",
    #             f"I_did_not_understand_you.I_will_set_your_{info}to_{incase_name}.",
    #         )
    #         check = False
    #         message_failed = True
    #         continue
    #
    #     if (rospy.get_time() - start_time) < ollama_timeout:
    #         response = rospy.wait_for_message(
    #             "ollama_response", OllamaResponse, timeout=ollama_response_timeout
    #         ).data
    #
    #         # success case
    #         if response and response.success:
    #             response_intent = response.intent.lower()
    #             if response_intent == publish_info:
    #                 info_output = rospy.wait_for_message(
    #                     "ollama_output", String, timeout=ollama_response_timeout
    #                 ).data
    #                 check = False if response_intent else True
    #                 continue
    #
    #         elif response and not response.success:
    #             if tries > MAX_TRIES:
    #                 p.exec_action(
    #                     "speak",
    #                     f"I_did_not_understand_you._Your_{info}_is_{incase_name}.",
    #                 )
    #                 check = False
    #                 message_failed = True
    #                 continue
    #
    #             else:
    #                 p.exec_action("speak", "I_did_not_understand_you.")
    #                 p.exec_action("speak", speech_text)
    #
    #                 start_time = rospy.get_time()
    #                 planner_intent_pub.publish(publish_info)
    #                 tries += 1
    #                 continue
    #     else:
    #         p.exec_action("speak", "Please_repeat_louder.")
    #         start_time = rospy.get_time()
    #         planner_intent_pub.publish(publish_info)
    #         time.sleep(1)
    #         continue
    #
    # if message_failed:
    #     info_output = incase_name

    # else:
    #     # confirmation from the user that the information is correct
    #     publish_info = f"affirm_deny"
    #     speech_text = f"Did_you_say_{info_output}?"
    #     affirm_check = True
    #     publish_info = affirm_check
    #
    #     p.exec_action("speak", speech_text)
    #     planner_intent_pub.publish(publish_info)
    #     time.sleep(1)
    #     listening_pub.publish(listening=True)
    #
    #     response = False
    #     check = True
    #     message_failed = False
    #     MAX_TRIES = 2
    #     overall_timeout = 120.0
    #     ollama_timeout = 10.0
    #     ollama_response_timeout = 5.0
    #     tries = 0
    #     response_affirmation = ''
    #
    #     initial_start_time = rospy.get_time()
    #     start_time = initial_start_time
    #     while affirm_check:
    #         if rospy.get_time() - initial_start_time > overall_timeout:
    #             p.exec_action(
    #                 "speak",
    #                 f"I_did_not_understand_you.I_will_set_your_{info}to_{incase_name}.",
    #             )
    #             affirm_check = False
    #             affirm_failed = True
    #             continue
    #
    #         if (rospy.get_time() - start_time) < ollama_timeout:
    #             response = rospy.wait_for_message(
    #                 "ollama_response", OllamaResponse, timeout=ollama_response_timeout
    #             ).data
    #
    #             # success case
    #             if response and response.success:
    #                 response_intent = response.intent.lower()
    #                 if response_intent == publish_info:
    #                     response_affirmation = rospy.wait_for_message(
    #                         "ollama_output", String, timeout=ollama_response_timeout
    #                     ).data
    #                     affirm_check = False if response_affirmation else True
    #                     continue
    #
    #                 else:
    #                     p.exec_action("speak", "I_did_not_understand_you.")
    #                     p.exec_action("speak", speech_text)
    #
    #                     start_time = rospy.get_time()
    #                     planner_intent_pub.publish(publish_info)
    #                     tries += 1
    #         else:
    #             p.exec_action("speak", "Please_repeat_louder.")
    #             planner_intent_pub.publish(publish_info)
    #
    
if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    obtain_person_name(p, "guest1", "name")

    p.end()
