#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import segmentation_models as sm
from cv_bridge import CvBridge
import os
import sys
import albumentations as A
import numpy as np
import cv2
import keras
import matplotlib.pyplot as plt
import webcolors


class color_id:
    
    def __init__(self,models_direct):
        self.BACKBONE = 'efficientnetb3'
        self.CLASSES = ['background', 'dress' , 'hair' , 'skin']
        self.activation = 'softmax'
        self.preprocess_input = sm.get_preprocessing(self.BACKBONE)
        self.weight_dir = models_direct+'best_model.h5'
        self.model  = sm.Unet(self.BACKBONE , classes = len(self.CLASSES) , activation = self.activation)
        self.model.load_weights(self.weight_dir)
        #self.pub1 = rospy.Publisher('dress' , Image , queue_size=10)
        #self.pub2 = rospy.Publisher('hair' , Image , queue_size=10)
        #self.pub3 = rospy.Publisher('skin' , Image , queue_size= 10)
        self.bridge = CvBridge()
        #self.pub_color_skin = rospy.Publisher('skin_color', Image , queue_size =10)
        #self.pub_color_hair = rospy.Publisher('hair_color', Image , queue_size =10)
        #self.pub_color_dress = rospy.Publisher('dress_color', Image , queue_size =10)
        self.transform = A.Compose([A.PadIfNeeded(320 , 320) ,A.Lambda(image=self.preprocess_input)])
        #self.sub1 = rospy.Subscriber('/webcam/image_raw' , Image , self.cb1)
    def imgmsg_to_cv2(self , img_msg):
        if img_msg.encoding != "bgr8":
            rospy.logerr("This node has been hardcoded to the 'bgr8' encoding")
        dtype = np.dtype("uint8") # Hardcode to 8 bits...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3),dtype=dtype, buffer=img_msg.data)
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        return image_opencv
    
    def Color(self , img_):
        #img_  = self.imgmsg_to_cv2(data)
        #img_ = cv2.cvtColor(img_  ,cv2.COLOR_BGR2RGB)
        img = self.transform(image = img_)['image']
        img = np.expand_dims(img , axis = 0)
        pr_mask = self.model.predict(img).round()
        pr_mask = pr_mask.astype(np.uint8)
        pr_mask = pr_mask.squeeze()
        #print(pr_mask[: , : , 1])
        #skin_filter = img_[pr_mask[: , : , 3] == 1]
        #hair_filter = img_[pr_mask[: , : , 2] == 1]
        #ress_filter = img_[pr_mask[: , : , 1] == 1]
        #print(dress_filter.shape)
        #for i in range(3):
        #    img_[: , : , i] = np.multiply(img_[: , : , i] , pr_mask[:  , : , 1])
        #img_ = np.multiply( img_[: , : , 1], pr_mask[ : , : , 1])
        #print(img_.shape)


        #print(np.mean(skin_filter , axis = 0))
        return pr_mask
