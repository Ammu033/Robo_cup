#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Image as IMAGE
import cv2
from cv_bridge import CvBridge
import os
from os import listdir
import numpy as np

from tensorflow.keras.models import Model, Sequential
from tensorflow.keras.layers import Input, Convolution2D, LocallyConnected2D, MaxPooling2D, Flatten, Dense, Dropout, Activation
from PIL import Image
from tensorflow.keras.preprocessing.image import load_img, save_img, img_to_array
from tensorflow.keras.applications.imagenet_utils import preprocess_input
from tensorflow.keras.preprocessing import image
import matplotlib.pyplot as plt
from tensorflow.keras.models import model_from_json
import sys
import copy
import random

def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3),dtype=dtype, buffer=img_msg.data)
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

class face_identifier:
	def __init__(self,database_direct,models_direct):
		self.face_detector_path = models_direct+"haarcascade_frontalface_default.xml"
		self.deepface_weights = models_direct+"VGGFace2_DeepFace_weights_val-0.9034.h5"
		self.emp_db = database_direct 
		self.model = self.model_func()
		self.bridge = CvBridge()
		self.img_target_size = (152 , 152)
		self.face_cascade = cv2.CascadeClassifier(self.face_detector_path)
		self.db_flag = False
		self.ret_emp()
		#self.sub  = rospy.Subscriber("/webcam/image_raw" , IMAGE , self.callback)
		#self.pub = rospy.Publisher("detected" , IMAGE , queue_size = 10)
		
	def model_func(self):
		base_model = Sequential()
		base_model.add(Convolution2D(32, (11, 11), activation='relu', name='C1', input_shape=(152, 152, 3)))
		base_model.add(MaxPooling2D(pool_size=3, strides=2, padding='same', name='M2'))
		base_model.add(Convolution2D(16, (9, 9), activation='relu', name='C3'))
		base_model.add(LocallyConnected2D(16, (9, 9), activation='relu', name='L4'))
		base_model.add(LocallyConnected2D(16, (7, 7), strides=2, activation='relu', name='L5') )
		base_model.add(LocallyConnected2D(16, (5, 5), activation='relu', name='L6'))
		base_model.add(Flatten(name='F0'))
		base_model.add(Dense(4096, activation='relu', name='F7'))
		base_model.add(Dropout(rate=0.5, name='D0'))
		base_model.add(Dense(8631, activation='softmax', name='F8'))
		base_model.load_weights(self.deepface_weights)
		model = Model(inputs=base_model.layers[0].input, outputs=base_model.layers[-3].output)
		return model
	
	
	def detectFace(self , image_path):
		img = cv2.imread(image_path)
		#faces = self.face_cascade.detectMultiScale(img, 1.3, 5)
		#if len(faces) > 0:
		#	x , y , w , h = faces[0]
		#	margin = 0
		#	x_margin = w * margin /100
		#	y_margin = h * margin/100
		#	if y - y_margin > 0 and y+h+y_margin < img.shape[1] and x-x_margin > 0 and x+w+x_margin < img.shape[0]:
		#		detected_face = img[int(y-y_margin):int(y+h+y_margin), int(x-x_margin):int(x+w+x_margin)]
		#	else:
		#		detected_face = img[int(y):int(y+h), int(x):int(x+w)]
		detected_face = cv2.resize(img , self.img_target_size)
		img_pixels = image.img_to_array(detected_face)
		img_pixels = np.expand_dims(img_pixels, axis = 0)
		img_pixels /= 255 
		return img_pixels
	
	def ret_emp(self):
		self.employees = dict()
		if (len(listdir(self.emp_db))) > 0:
			self.db_flag = True
		else :
			self.db_flag = False
		for img_name in listdir(self.emp_db):
			emp , _ = img_name.split('.')
			img_path = self.emp_db + img_name
			img = self.detectFace(img_path)
			emp_rep = self.model.predict(img)[0]
			self.employees[emp] = emp_rep
			print("Retieved Updated People Representations")
			
	def EuclideanDistance(self , src , test ) :
		euclidean_distance = src - test
		euclidean_distance = np.sum(np.multiply(euclidean_distance, euclidean_distance))
		euclidean_distance = np.sqrt(euclidean_distance)
		return euclidean_distance

	def l2(self , x ) :
		return x / np.sqrt(np.sum(np.multiply(x, x)))

	
	def register_new_face(self , img_copy, x , y , w  ,h):
		print('New Face Detected')
		#print('Tell your name')
		#os.system('mpg321 name_promp.mp3')
		#new_img_name = str(input())
		new_img_name = "".join(random.choices("1234567890", k=5))
		filename = self.emp_db + new_img_name + '.jpg'
		cv2.imwrite(filename, img_copy[int(y):int(y+h), int(x):int(x+w)])
		self.ret_emp()
		return new_img_name
	
				
	def name_teller(self, data , x , y , w , h):
		#img = imgmsg_to_cv2(data)
		img_copy = copy.deepcopy(data)
		#faces = self.face_cascade.detectMultiScale(img , 1.3 , 5)
		#for (x , y , w , h ) in faces: 
		#if w > 100 :
		detected_face = data[int(y):int(y+h), int(x):int(x+w)] #crop detected face
		detected_face = cv2.resize(detected_face, self.img_target_size) #resize to 152x152
		##img_copy = img
		img_pixels = image.img_to_array(detected_face)
		img_pixels = np.expand_dims(img_pixels, axis = 0)
		img_pixels /= 255
		cap_rep = self.model.predict(img_pixels )[0]
		distances = []
		#if (self.db_flag == False) :
		#	self.register_new_face(img_copy ,x , y , w  ,h )
		#	self.ret_emp()
		employee_name = None
		for i in self.employees : 
			employee_name = i
			src_rep = self.employees[i]
			distance = self.EuclideanDistance(self.l2(cap_rep) , self.l2(src_rep))
			distances.append(distance)
		index = 0
		is_found = False
		color = (200 , 0 , 0 )
		name = None 
		for i in self.employees:
			employee_name = i
			if index == np.argmin(distances):
				if distances[index] <= 1.0:
					is_found = True
					color  = (0 , 200 ,0)
					name = employee_name
					rospy.loginfo("detected: " + str(employee_name)  + "(" + str(distances[index]) + ")")
				#if distances[index] > 0.70 :
					#self.register_new_face(img_copy , x , y , w , h )
					#self.ret_emp()
			index = index + 1
		#img = cv2.rectangle(img , (x,y) , (x+w , y+ h) , color , 3)
		#if is_found == False:
		#	self.register_new_face(img_copy , x, y , w, h )
		#	self.ret_emp()
		return name  , is_found



