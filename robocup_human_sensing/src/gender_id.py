#!/usr/bin/env python
import tensorflow as tf
from tensorflow.keras.models import Model, Sequential
from tensorflow.keras.layers import Convolution2D, Flatten, Activation
from vgg import baseModel
import cv2
import numpy as np

class gender_identifier:
    def __init__(self,models_direct):
        self.gender_path = models_direct+"gender_model_weights.h5"
        self.model = self.model_func()
        self.target_size = (224 , 224)
        self.gender_list = ['Woman' , 'Man']

    def model_func(self):
        model = baseModel()
        classes = 2
        model_output = Sequential()
        model_output = Convolution2D(classes , (1,1) , name='preditions')(model.layers[-4].output)
        model_output = Flatten()(model_output)
        model_output = Activation("softmax")(model_output)
        gender_model = Model(inputs=model.input , outputs=model_output)
        gender_model.load_weights(self.gender_path)
        return gender_model
    
    def gender_teller(self , data , x , y , w , h ):
        detected_face = data[int(y): int(y+h) , int(x) : int(x+h)]
        detected_face = cv2.resize(detected_face , self.target_size)
        gender_p = self.model.predict(detected_face , verbose = 0)[0, : ]
        return self.gender_list[np.argmax(gender_p)]
