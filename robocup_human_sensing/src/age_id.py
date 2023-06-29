#!/usr/bin/env python
#import rospy
#from sensor_msgs.msg import Image
import tensorflow as tf
from tensorflow.keras.models import Model, Sequential
from tensorflow.keras.layers import Convolution2D, Flatten, Activation
from vgg import baseModel
import cv2
import numpy as np

class age_identifier:
    def __init__(self,models_direct):
        self.age_path = models_direct+"age_model_weights.h5"
        self.model = self.model_func()
        self.target_size = (224 , 224)


    def model_func(self):
        model = baseModel()
        classes = 101
        model_output = Sequential()
        model_output = Convolution2D(classes , (1,1) , name='preditions')(model.layers[-4].output)
        model_output = Flatten()(model_output)
        model_output = Activation("softmax")(model_output)
        age_model = Model(inputs=model.input , outputs=model_output)
        age_model.load_weights(self.age_path)
        return age_model
    
    def findApparentAge(self, age_predictions):
        output_indexes  = np.array(list(range(0 , 101)))
        apparent_age = np.sum(age_predictions *output_indexes)
        return apparent_age
    
    def age_teller(self, data , x , y , w ,h ):
        detected_face = data[int(y) : int(y + h ) , int(x) : int(x+w)]
        detected_face = cv2.resize(detected_face , self.target_size)
        age_p  = self.model.predict(detected_face, verbose=0)[0,  :]
        return self.findApparentAge(age_p)
    

