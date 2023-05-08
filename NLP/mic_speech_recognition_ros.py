import speech_recognition as sr
import whisper
import queue
import threading
import torch
import numpy as np
import rospy 
import time
from std_msgs.msg import String

model=rospy.get_param("/lcastor_nlp/speech_recogn_model","tiny") # options are ["tiny","base", "small","medium"]
language=rospy.get_param("/lcastor_nlp/speech_recogn_language","english") #english language by default
energy=rospy.get_param("/lcastor_nlp/speech_recogn_energy",300) #Energy level for mic to detect
dynamic_energy=rospy.get_param("/lcastor_nlp/speech_recogn_dyn_energy_flag",False) #flag to enable dynamic energy
pause=rospy.get_param("/lcastor_nlp/speech_recogn_pause_time",0.8) #Pause time before entry ends
pub_hz=0.01 #main loop frequency

class speech_class:
    def __init__(self): 
        self.audio_model = whisper.load_model(model)
        self.audio_queue = queue.Queue()
        self.transcription = ""
        self.detection=False #Flag to know if a speech have been recognized
        threading.Thread(target=self.record_audio,
                         args=(energy, pause, dynamic_energy)).start()
        threading.Thread(target=self.transcribe_speech).start()
    
    def record_audio(self, energy, pause, dynamic_energy):
        #load the speech recognizer and set the initial energy threshold and pause threshold
        r = sr.Recognizer()
        r.energy_threshold = energy
        r.pause_threshold = pause
        r.dynamic_energy_threshold = dynamic_energy
    
        with sr.Microphone(sample_rate=16000) as source:
            print("Say something!")
            while True and not rospy.is_shutdown():
                #get and save audio to wav file
                audio = r.listen(source)
                torch_audio = torch.from_numpy(np.frombuffer(audio.get_raw_data(), np.int16).flatten().astype(np.float32) / 32768.0)
                audio_data = torch_audio
                self.audio_queue.put_nowait(audio_data)
           

    def transcribe_speech(self):
        while True and not rospy.is_shutdown():
            audio_data = self.audio_queue.get()
            result = self.audio_model.transcribe(audio_data,language=language) #english language by default
            self.transcription = result["text"]
            print("You said: ",self.transcription)
            self.detection=True
            
    def nlp(self):
        print("Processing the text")
        return self.transcription
        
            
if __name__ == '__main__':
    time_init=time.time()  
    # Initialize our node       
    speech=speech_class()
    rospy.init_node('speech_recognition',anonymous=True)
    # Setup and call subscription
    #rospy.Subscriber('human_detection',hri_msg,hri.safety_callback)
    # Setup publisher and subscription
    pub = rospy.Publisher('speech_command', String, queue_size=1)
    #Rate setup
    rate = rospy.Rate(1/pub_hz) # main loop frecuency in Hz
    while not rospy.is_shutdown():
        #print("Detection",speech.detection)
        if speech.detection==True:
            speech.command=speech.nlp()
            pub.publish(speech.command)
            speech.detection=False
        rate.sleep() 
        