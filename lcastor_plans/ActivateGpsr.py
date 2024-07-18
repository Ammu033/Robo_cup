import os
import sys
from ollamamessages.msg import WhisperTranscription, WhisperListening
from ollamamessages.msg import WhisperListening
from ollamamessages.srv import OllamaCall, OllamaCallResponse

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

no_speech_thresh = 0.2 

def send_to_gpsr_service():
    raise NotImplementedError

def call_whisper(listening_pub,  tries):
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

def run_gpsr_sevice(listening_pub, text):
   


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
    p.exec_action("gotoRoom", "r_instructionpoint")
    listening_pub = rospy.Publisher("/stt/listening", WhisperListening, queue_size=1)

    tries = 0
    success = False
    heard_speech = None

    while (tries <= 2 and success == False) or (success == True):
        success, heard_speech = call_whisper(listening_pub, tries)
        tries += 1 

    if heard_speech is None:
        p.exec_action('speak', 'Sadly_I_could_not_catch_what_you_said,_have_a_nice_day')
        return

    try:
        service_call = rospy.ServiceProxy("/gpsr/task_decomposition", OllamaCall)
        response = service_call(input = heard_speech)
        print(response)
    except Exception as e:
        print("Gpsr failed: ", str(e))
        p.exec_action('speak', 'Sorry,_the_tasks_might_have_been_difficult_to_understand,_have_a_nice_day')
            
    rospy.loginfo("Successfully sent, generating GPSR, stopping listning.")
    listening_pub.publish(listening = False)   


if __name__ == "__main__":
    p = PNPCmd()
    p.begin()
    gpsr()
    p.end()
