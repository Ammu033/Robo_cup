#! /usr/bin/env python3

import speech_recognition as sr
import whisper
import queue
import threading
import torch
import numpy as np
import rospy 
import time
from std_msgs.msg import String, Bool

model=rospy.get_param("/stt/speech_recogn_model","small") # options are ["tiny","base", "small","medium"]
language=rospy.get_param("/stt/speech_recogn_language","english") #english language by default
energy=rospy.get_param("/stt/speech_recogn_energy",400) #Energy level for mic to detect
dynamic_energy=rospy.get_param("/stt/speech_recogn_dyn_energy_flag",False) #flag to enable dynamic energy
#phrase_time_limit=rospy.get_param("/stt/phrase_time_limit",4) #Time between sentences
pause=rospy.get_param("/stt/speech_recogn_pause_time",0.8) #Pause time before entry ends
pub_hz=0.01 #main loop frequency

class speech_class:
    def __init__(self): 
        self.audio_model = whisper.load_model(model)
        self.audio_queue = queue.Queue(maxsize=1)
        self.transcription = ""
        self.detection=False #Flag to know if an user speech have been recognized
        self.send_speech=False  #Flag to know if planner requires speech recognition and rasa
        self.sr=threading.Thread(target=self.record_audio,
                         args=(energy, pause, dynamic_energy))
        self.ts=threading.Thread(target=self.transcribe_speech)
        self.sr.start()
        self.ts.start()
    
    def record_audio(self, energy, pause, dynamic_energy):
        #load the speech recognizer and set the initial energy threshold and pause threshold
        r = sr.Recognizer()
        r.energy_threshold = energy
        r.pause_threshold = pause
        r.dynamic_energy_threshold = dynamic_energy
            
        with sr.Microphone(sample_rate=16000) as source:
            #print("Calibrating Ambient Noise For 5 Seconds...")
            #r.adjust_for_ambient_noise(source, duration=1)
            print("Ready")
            while True and not rospy.is_shutdown():
                #print("MAIN",self.send_speech)
                audio = r.listen(source)
                print('I am here')
                torch_audio = torch.from_numpy(np.frombuffer(audio.get_raw_data(), np.int16).flatten().astype(np.float32) / 32768.0)
                audio_data = torch_audio
                if self.send_speech==True: #only consider new audio if planner have requested speech recognition
                    self.audio_queue.put(audio_data)
                    print("Listening...")
                else:
                    print("Not listening")
                

    def transcribe_speech(self):
        while True and not rospy.is_shutdown():
            audio_data = self.audio_queue.get()
            result = self.audio_model.transcribe(audio_data,language=language) #english language by default
            self.transcription = result["text"]
            print("You said: ",self.transcription)
            self.detection=True
    
    def planner_intention_callback(self,msg):
        self.send_speech=True
        print("Say something")
        
    
    def rasa_confirmation_callback(self,msg):
        #If rasa returned something, then disable sending user speech to rasa_ros_bridge
        self.send_speech=False
        print("Speech recognition disabled")                   
            
if __name__ == '__main__':
    time_init=time.time()  
    # Initialize our node       
    speech=speech_class()
    rospy.init_node('speech_to_text',anonymous=True)
    # Setup publisher and subscription
    pub = rospy.Publisher('user_speech', String, queue_size=1)
    rospy.Subscriber('planner_intention',String,speech.planner_intention_callback) 
    rospy.Subscriber('rasa_confirmation',Bool,speech.rasa_confirmation_callback)
    #Rate setup
    rate = rospy.Rate(1/pub_hz) # main loop frecuency in Hz
    while not rospy.is_shutdown():
        #print("Detection",speech.detection)
        if speech.detection==True:
            pub.publish(speech.transcription)
            speech.detection=False
        rate.sleep() 
        
