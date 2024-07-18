import os
import sys
from lcastor_plans import AskConfirmation
from ollamamessages.msg import WhisperTranscription, WhisperListening
from ollamamessages.msg import WhisperListening
from ollamamessages.srv import OllamaCall, OllamaCallResponse
from AskConfirmation import AskConfirmation

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


def call_whisper(listening_pub,  tries, no_speech_thresh=0.2):
    listening_pub.publish(listening=False)
    if tries == 0: 
        p.exec_action('speak', 'Im_ready_for_my_instructions')
    else:
        p.exec_action('speak', 'sorry,_could_you_repeat_that_more_closely')
    
    start_time = rospy.get_time()
    listening_pub.publish(listening=True)
    transcription = rospy.wait_for_message('/stt/transcription', WhisperTranscription).data
    listening_pub.publish(listening=False)
    text = transcription['text']
    if transcription['no_speech_prob'] <= no_speech_thresh:
        return True, text
    return False, text   


def gpsr(p):
    pause = 0.8
    energy = 4000
    dynamic_energy = False
    no_speech_thresh = 0.2
    
    rospy.set_param("/stt/use_ollama", False)
    rospy.set_param("/stt/speech_recogn_pause_time", pause)
    rospy.set_param("/stt/speech_recogn_energy", energy) 
    rospy.set_param("/stt/speech_recogn_dyn_energy_flag", dynamic_energy)
    rospy.set_param("/stt/speech_confidence_thresh", no_speech_thresh)
    rospy.loginfo(f'GPSR PARAMS - Pause: {pause}, Energy: {energy}, Dynamic Energy: {dynamic_energy}, No Speech Threshold: {no_speech_thresh}')

    # 1. Wait for the door open
    p.exec_action("speak", "Can_you_please_open_the_door_for_me_?")
    while(not p.get_condition("isDoorOpen")): time.sleep(0.1)
    p.exec_action("speak", "Thank_you_for_opening_the_door")
    p.exec_action("navigateForward", "4")

    # TODO: MUST SET THE INSTRUCTION POINT!!
    p.exec_action("gotoRoom", "r_inspectionpoint")
    listening_pub = rospy.Publisher("/stt/listening", WhisperListening, queue_size=1)

    tries = 0
    success = False
    heard_speech = None

    while tries <= 2:
        success, heard_speech = call_whisper(listening_pub, tries, no_speech_thresh)
        if success == True:
            break
        tries += 1 

    if heard_speech is None:
        p.exec_action('speak', 'Sadly_I_could_not_catch_what_you_said,_have_a_nice_day')
        return
    
    success, response = AskConfirmation(
        p, 
        speech_text="Did_I_hear_you_say_"+heard_speech.replace(' ','_')+"?", 
        cannot_hear_text="please_say_yes_if_that_is_correct.",
        max_tries=0
    )
    
    if not success: 
        p.exec_action('speak', 'Sorry,_I_did_not_understand_your_confirmation,_have_a_nice_day')
        return
    
    if response != 'yes':
        p.exec_action('speak', 'Sorry,_It_seems_I_misunderstood,_have_a_nice_day')
        return

    try:
        service_call = rospy.ServiceProxy("/gpsr/task_decomposition", OllamaCall)
        response = service_call(input = heard_speech)
        print(response)
    except Exception as e:
        print("GPSR failed: ", str(e))
        p.exec_action('speak', 'Sorry,_the_tasks_might_have_been_difficult_to_understand,_have_a_nice_day')
            
    rospy.loginfo("Successfully sent, generating GPSR, stopping listening.")
    listening_pub.publish(listening = False)   


if __name__ == "__main__":
    p = PNPCmd()
    p.begin()
    gpsr()
    p.end()
