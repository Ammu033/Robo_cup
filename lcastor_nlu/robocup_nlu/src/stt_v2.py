#! /usr/bin/env python3

import speech_recognition as sr
import whisper
import queue
import threading
import torch
import numpy as np
import rospy 
import time
from std_msgs.msg import String

model=rospy.get_param("/stt/speech_recogn_model","medium") # options are ["tiny","base", "small","medium"]
language=rospy.get_param("/stt/speech_recogn_language","english") #english language by default
energy=rospy.get_param("/stt/speech_recogn_energy",400) #Energy level for mic to detect
dynamic_energy=rospy.get_param("/stt/speech_recogn_dyn_energy_flag",False) #flag to enable dynamic energy
pause=rospy.get_param("/stt/speech_recogn_pause_time",0.8) #Pause time before entry ends
pub_hz=0.01 #main loop frequency

class speech_class:
    def __init__(self): 
        self.audio_model = whisper.load_model(model)
        self.audio_queue = queue.Queue()
        self.transcription = ""
        self.detection=False #Flag to know if an user speech have been recognized
        self.start_listening=False
        threading.Thread(target=self.record_audio,
                         args=(energy, pause, dynamic_energy)).start()
        threading.Thread(target=self.transcribe_speech).start()
    
    def record_audio(self, energy, pause, dynamic_energy):
        #load the speech recognizer and set the initial energy threshold and pause threshold
        r = sr.Recognizer()
        r.energy_threshold = energy
        r.pause_threshold = pause
        r.dynamic_energy_threshold = dynamic_energy
    
        with sr.Microphone(sample_rate=16000) as source: ###recording ambient noise and adjusting the values for this###

            print("Calibrating Ambient Noise For 5 Seconds...")
            r.adjust_for_ambient_noise(source, duration=5)
            
            print("Say something!")
            while True and not rospy.is_shutdown():
                #get and save audio to wav file
                
                audio = r.listen(source, phrase_time_limit=5)  ###set a maximum phrase time limit (e.g., 5 seconds)###
                
                torch_audio = torch.from_numpy(np.frombuffer(audio.get_raw_data(), np.int16).flatten().astype(np.float32) / 32768.0)
                audio_data = torch_audio
                self.audio_queue.put_nowait(audio_data)
           

    def transcribe_speech(self):
        while True and not rospy.is_shutdown():
            audio_data = self.audio_queue.get()
            result = self.audio_model.transcribe(audio_data,language=language) #english language by default
            self.transcription = result["text"]
            print("You said: ",self.transcription)

            ###code waits for the following phrase before setting the detection statement to TRUE which runs the loop###
            if self.start_listening:
                if "Robot Listen" in self.transcription:
                    self.detection = True
                    
            ###if the phrase has been detected by the system "phrase detected" is displayed as a debug, loop starts with the "calibrating noise" code###
            if self.detection:
                self.start_listening = True
                print("Phrase Detected!")
                self.detection = False
                    
            
if __name__ == '__main__':
    time_init=time.time()  
    # Initialize our node       
    speech=speech_class()
    rospy.init_node('speech_to_text',anonymous=True)
    # Setup publisher and subscription
    pub = rospy.Publisher('user_speech', String, queue_size=1)
    #Rate setup
    rate = rospy.Rate(1/pub_hz) # main loop frecuency in Hz
    while not rospy.is_shutdown():
        #print("Detection",speech.detection)
        if speech.detection:
            pub.publish(speech.transcription)
            speech.start_listening = False
        rate.sleep() 
        
