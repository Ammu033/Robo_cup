#!/usr/bin/env python3

from ollamamessages.msg import WhisperTranscription, WhisperListening
from ollamamessages.srv import OllamaCall, OllamaCallResponse
from std_msgs.msg import String

import speech_recognition as sr
import threading
import tempfile
import requests
import rospy 
import time
import json
import os

whisper_api_url = rospy.get_param("/stt/whisper_api_url", "127.0.0.1:9000")
pause = rospy.get_param("/stt/speech_recogn_pause_time", 0.8)
energy = rospy.get_param("/stt/speech_recogn_energy", 4000) 
dynamic_energy = rospy.get_param("/stt/speech_recogn_dyn_energy_flag", False)
no_speech_thresh = rospy.get_param("/stt/speech_confidence_thresh", 0.2)

class WhisperWrapper:

    listening = False

    def __init__(self) -> None:
        self.transcription_pub = rospy.Publisher("/stt/transcription", WhisperTranscription, queue_size = 1)
        self.listening_sub = rospy.Subscriber("/stt/listening", WhisperListening, self.listening_sub_cb)
        self.listening_pub = rospy.Publisher("/stt/listening", WhisperListening, queue_size = 1)

        self.record_audio(pause, energy, dynamic_energy)

    def listening_sub_cb(self, set_listening):
        rospy.loginfo("Set listening = %s" % str(set_listening.listening))
        self.listening = set_listening.listening

    def record_audio(self, pause, energy, dynamic_energy):
        recogniser = sr.Recognizer()
        recogniser.energy_threshold = energy
        recogniser.pause_threshold = pause
        recogniser.dynamic_energy_threshold = dynamic_energy

        with sr.Microphone() as microphone:
            # recogniser.adjust_for_ambient_noise(microphone)
            rospy.loginfo("Listening...")
            while True and not rospy.is_shutdown():
                audio = recogniser.listen(microphone)

                if not self.listening:
                    rospy.loginfo("I heard something but I'm stopping here because we've been set to not listen")
                    continue

                with tempfile.NamedTemporaryFile(mode = "wb", suffix = ".wav", delete = False) as f:
                    audio_path = f.name
                    f.write(audio.get_wav_data())

                rospy.loginfo("I heard something... Written to %s" % audio_path)
                req = requests.post(
                    "http://%s/asr?output=json" % whisper_api_url,
                    files = {"audio_file": open(audio_path, "rb")}
                )
                os.remove(audio_path)
                o = req.json()
                if o["text"] != "":
                    no_speech_prob = o["segments"][0]["no_speech_prob"]
                    rospy.loginfo("Transcribed '%s' (p=%.2f)" % (o["text"], no_speech_prob))
                    self.transcription_pub.publish(
                        text = o["text"],
                        language = o["language"],
                        temperature = o["segments"][0]["temperature"],
                        avg_logprob = o["segments"][0]["avg_logprob"],
                        compression_ratio = o["segments"][0]["compression_ratio"],
                        no_speech_prob = no_speech_prob
                    )

                    if no_speech_prob < no_speech_thresh:
                        self.run_ollama(o["text"])
                    else:
                        rospy.loginfo("Skipped due to low confidence it's actually speech. (p=%.2f)" % no_speech_prob)
                else:
                    rospy.loginfo("Empty value for transcription recieved")

    def run_ollama(self, text):
        try:
            service_call = rospy.ServiceProxy("/stt/ollamacall", OllamaCall)
            response = service_call(input = text)
            print(response)
        except Exception as e:
            print("Ollama failed: ", str(e))
        else:        
            rospy.loginfo("We've successfully sent something to ollama, so let's stop listening.")
            self.listening_pub.publish(listening = False)      

if __name__ == "__main__":
    rospy.init_node("whisper_wrapper")
    whisperwrapper = WhisperWrapper()
    


