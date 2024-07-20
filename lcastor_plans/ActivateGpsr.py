import os
import sys
from ollamamessages.msg import WhisperTranscription, WhisperListening
from ollamamessages.srv import OllamaCall
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


def call_whisper(listening_pub,  tries, no_speech_thresh=0.2, text=None):

    listening_pub.publish(listening=False)
    if text == 'waiting':
        time.sleep(2)
        p.exec_action('speak', 'say,_new_task,_when_ready')
        time.sleep(2)
    elif tries == 0: 
        p.exec_action('speak', 'I_am_ready_for_my_instructions')
    else:
        p.exec_action('speak', 'sorry,_could_you_repeat_that_more_closely')
    
    start_time = rospy.get_time()
    listening_pub.publish(listening=True)
    transcription = rospy.wait_for_message('/stt/transcription', WhisperTranscription)
    listening_pub.publish(listening=False)
    text = transcription.text
    if transcription.no_speech_prob <= no_speech_thresh:
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
    p.exec_action("moveHead", "0.0_0.0")
    p.exec_action("speak", "Can_you_please_open_the_door_for_me_?")
    while(not p.get_condition("isDoorOpen")): time.sleep(0.1)
    p.exec_action("speak", "Thank_you_for_opening_the_door")
    p.exec_action("navigateForward", "7")
    n = 0
    while n < 3:
        p.exec_action("gotoRoom", "r_inspectionpoint")
        listening_pub = rospy.Publisher("/stt/listening", WhisperListening, queue_size=1)

        tries = 0
        success = False
        heard_speech = None
        fail = False
        response = None


        while True:
            success, heard_speech = call_whisper(listening_pub, 0, no_speech_thresh, 'waiting')
            if 'new task' in  heard_speech.lower():
                break

        
        while tries <= 2:
            success, heard_speech = call_whisper(listening_pub, tries, no_speech_thresh)
            if success == True:
                break
            tries += 1 

        if heard_speech is None:
            fail = True
        else: 
            p.exec_action('speak', "Did_I_hear_you_say_"+heard_speech.replace(' ','_')+"?")
            success = True
            response = "yes"
            time.sleep(3)
            # success, response = AskConfirmation(
            #     p, 
            #     speech_text="Did_I_hear_you_say_"+heard_speech.replace(' ','_')+"?", 
            #     cannot_hear_text="please_say_yes_if_that_is_correct.",
            #     max_tries=0
            # )
            p.exec_action('speak','thanks_for_confirming')
        
        if not success: 
            fail = True
        else:
            if response != 'yes':
                fail = True
        
        if fail == True:
            p.exec_action('speak', 'Sorry,_it_seems_i_couldnt_understand_please_type_out_the_gpsr_command')
            heard_speech = input("Please type out the gpsr command: ")

        try:
            service_call = rospy.ServiceProxy("/gpsr/task_decomposition", OllamaCall)
            response = service_call(input = heard_speech)
            listening_pub.publish(listening = False)
            print(response)
        except Exception as e:
            print("GPSR failed: ", str(e))
            p.exec_action('speak', 'Sorry,_the_tasks_might_have_been_difficult_to_understand')
        rospy.loginfo("Successfully sent, generating GPSR, stopping listening.")
        n += 1   


if __name__ == "__main__":
    p = PNPCmd()
    p.begin()
    gpsr(p)
    p.end()
