import os
import sys
from ollamamessages.msg import WhisperListening, OllamaResponse
from std_msgs.msg import String


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

OVERALL_TIMEOUT = 30.0
RESPONSE_TIMEOUT = 6.0
MAX_TRIES = 2 

def speech_for_whisper(p, listening_pub, plan_intent_pub, publish_info, speech_texts):
    listening_pub.publish(listening=False)
    if isinstance(speech_texts, str):
        p.exec_action("speak", speech_texts)
    elif isinstance(speech_texts, list):
        for speech in speech_texts:
            p.exec_action("speak", speech)
    plan_intent_pub.publish(publish_info)
    time.sleep(1)
    listening_pub.publish(listening=True)
    start_time = rospy.get_time()
    return start_time

def request_ollama(
        p,
        info,
        publish_info,
        speech_text,
        default_info,
        cannot_hear_text = "please_repeat_that",
        tries = MAX_TRIES,
        timeout = OVERALL_TIMEOUT,
        response_timeout = RESPONSE_TIMEOUT,
        retry_timout = RESPONSE_TIMEOUT
    ):
    get_is_use_ollama = rospy.get_param("/stt/use_ollama")
    rospy.set_param("/stt/use_ollama", True)

    # preparing publishers
    listening_pub = rospy.Publisher("/stt/listening", WhisperListening, queue_size=1)
    plan_intent_pub = rospy.Publisher("/planner_intention", String, queue_size=1, latch=True)

    plan_intent_pub.publish(publish_info)
    time.sleep(1)
    listening_pub.publish(listening=True)
    response = False
    check = True
    message_failed = False
    info_output = None
    tries = 0

    # request from user text
    p.exec_action("speak", speech_text)
    initial_start_time = rospy.get_time()
    start_time = initial_start_time

    while check:
        if rospy.get_time() - initial_start_time > timeout:
            start_time = speech_for_whisper(p, listening_pub, plan_intent_pub, publish_info, [f"I_did_not_understand_you.I_will_set_your_{info}_to_{default_info}."])
            check = False
            message_failed = True
            continue

        if (rospy.get_time() - start_time) < retry_timout:
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

                    info_output = info_output.replace(' ', '_')
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
                    speech_for_whisper(p, listening_pub, plan_intent_pub, publish_info, ["I_did_not_understand_you.", speech_text])
                    tries += 1
                    continue
        else:
            start_time = speech_for_whisper(p, listening_pub, plan_intent_pub, publish_info, [cannot_hear_text])
            continue

    # turning off listening after the response are complete
    listening_pub.publish(listening=False)

    if not info_output or message_failed:
        info_output = default_info

    success = not message_failed
    rospy.loginfo(f'info captured: {info_output}')
    rospy.loginfo(f'success status: {success}')

    rospy.set_param("/stt/use_ollama", get_is_use_ollama)
    return (success, info_output)


if __name__ == "__main__":

    p = PNPCmd()
    p.begin()

    info = 'name'
    topic = 'guest_name'
    speech_text = 'What_is_your_name_today?'
    default_info = 'Sam'

    request_ollama(p, info, topic, speech_text, default_info)

    p.end()
