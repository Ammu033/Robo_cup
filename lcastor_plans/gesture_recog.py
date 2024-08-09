#!/usr/bin/env python

import rospy
import ros_numpy
from sensor_msgs.msg import Image
from ultralytics import YOLO
import cv2
import random
import numpy as np
import tf
# import tf2_msgs.msg
from geometry_msgs.msg import Pose, Pose2D
MINIMUM_FRAMES = 30
class yolo_trial:
    def __init__(self):
        self.model  = YOLO('yolov8n-pose.pt')
        self.pub = rospy.Publisher('/from_yolo' , Image , queue_size=10)
        self.color_r = list(range(0,255 , 20))
        self.color_g = list(range(0,255, 20))
        self.color_b = list(range(0 , 255 , 20)) 
        random.shuffle(self.color_r)
        random.shuffle(self.color_g)
        random.shuffle(self.color_b)
        self.guest_location  = Pose()
        # self.l = tf.TransformListener()
        # self.robot_pose = Pose2D()
        self.person_location_x = None
        self.person_location_y = None
        self.person_location_z = None
        print('Model Loaded ')
        print('Node Started')
        self.multi_message = {}
        self.principal_point_x = 320.5
        self.principal_point_y = 240.5
        self.focal_length_x = 554.254691191187
        self.focal_length_y = 554.254691191187
        self.image_sub = rospy.Subscriber('/xtion/rgb/image_raw' , Image , self.image_cb)


    def image_cb(self,  data):
        image_np  =ros_numpy.numpify(data)
        results = self.model.track(source=  image_np , stream = False , verbose = False)
        
        for r in results:
            if r.boxes.id is not None and hasattr(r.boxes , 'id'):
                ids = r.boxes.id.cpu().numpy()
                keypoints  =r.keypoints.xy.cpu().numpy()
                for index, keypoint_ in enumerate(keypoints):
                    results_message={}
                    results_message['left_shoulder_1']=keypoint_[5,0]
                    results_message['left_shoulder_2']=keypoint_[5,1]
                    results_message['right_shoulder_1']=keypoint_[6,0]
                    results_message['right_shoulder_2']=keypoint_[6,1]
                    results_message['left_arm_1']=keypoint_[7,0]
                    results_message['left_arm_2']=keypoint_[7,1]
                    results_message['right_arm_1']=keypoint_[8,0]
                    results_message['right_arm_2']=keypoint_[8,1]
                    results_message['left_hand_1']=keypoint_[9,0]
                    results_message['left_hand_2']=keypoint_[9,1]
                    results_message['right_hand_1']=keypoint_[10,0]
                    results_message['right_hand_2']=keypoint_[10,1]
                    results_message['left_waist_1']=keypoint_[11,0]
                    results_message['left_waist_2']=keypoint_[11,1]
                    results_message['right_waist_1']=keypoint_[12,0]
                    results_message['right_waist_2']=keypoint_[12,1]
                    image_np = cv2.circle(image_np , (int(keypoint_[5,0]) , int(keypoint_[5,1])) , radius=5, color = (self.color_r[index] , self.color_g[index] ,self.color_b[index])  , thickness = -1)
                    image_np = cv2.circle(image_np ,(int(keypoint_[6,0]) , int(keypoint_[6,1])) , radius=5, color = (self.color_r[index] , self.color_g[index] ,self.color_b[index])  , thickness = -1)
                    image_np = cv2.circle(image_np ,(int(keypoint_[7,0]) , int(keypoint_[7,1])) , radius=5 , color = (self.color_r[index] , self.color_g[index] ,self.color_b[index])  , thickness = -1)
                    image_np = cv2.circle(image_np ,(int(keypoint_[8,0]) , int(keypoint_[8,1])) , radius=5 , color = (self.color_r[index] , self.color_g[index] ,self.color_b[index])  , thickness = -1)
                    image_np = cv2.circle(image_np ,(int(keypoint_[9,0]) , int(keypoint_[9,1])) , radius=5 , color = (self.color_r[index] , self.color_g[index] ,self.color_b[index])  , thickness = -1)
                    image_np = cv2.circle(image_np ,(int(keypoint_[10,0]) , int(keypoint_[10,1])) , radius=5 , color = (self.color_r[index] , self.color_g[index] ,self.color_b[index])  , thickness = -1)

                    keypoints = [
                    [results_message['right_hand_1'], results_message['right_hand_2']],
                    [results_message['right_arm_1'], results_message['right_arm_2']], 
                    [results_message['right_shoulder_1'], results_message['right_shoulder_2']], 
                    [results_message['right_waist_1'], results_message['right_waist_2']], 
                    [results_message['left_waist_1'], results_message['left_waist_2']],      
                    [results_message['left_shoulder_1'], results_message['left_shoulder_2']],
                    [results_message['left_arm_1'], results_message['left_arm_2']],          
                    [results_message['left_hand_1'], results_message['left_hand_2']],]

                    gesture_detected = False
                    if keypoints[0][1] < keypoints[2][1]:
                        a = np.array([keypoints[0][0], keypoints[0][1]])
                        b = np.array([keypoints[1][0], keypoints[1][1]])
                        c = np.array([keypoints[2][0], keypoints[2][1]])
                        ba = a - b
                        bc = c - b
                        if np.linalg.norm(ba) != 0 and np.linalg.norm(bc) != 0:
                            cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
                            angle = np.degrees(np.arccos(cosine_angle))
                            # print(angle)
                            if angle > 30 : #and angle < 130:
                                gesture_detected = True
                    elif keypoints[7][1] < keypoints[5][1]:
                        h = np.array([keypoints[7][0], keypoints[7][1]])
                        g = np.array([keypoints[6][0], keypoints[6][1]])
                        f = np.array([keypoints[5][0], keypoints[5][1]])
                        gh = h - g
                        gf = f - g

                        if np.linalg.norm(gh) != 0 and np.linalg.norm(gf) != 0:
                            cosine_angle = np.dot(gh, gf) / (np.linalg.norm(gh) * np.linalg.norm(gf))
                            angle = np.degrees(np.arccos(cosine_angle))
                            # print(angle)
                            if angle > 30 : #and angle < 130:
                                gesture_detected =True
                    
                    # print(gesture_detected)
                    if gesture_detected : 
                        try : 
                            self.multi_message[ids[index]] += 1
                        except KeyError: 
                            self.multi_message[ids[index]] = 1
                        # print(multi_message[ids[index]])
                        print(self.multi_message[ids[index]])
                        if self.multi_message[ids[index]] > MINIMUM_FRAMES:
                            self.multi_message[ids[index]] = 0
                            print("Gesture Detected")
                            depth_image = rospy.wait_for_message('/xtion/depth/image' ,Image, timeout=5.0 )
                            # x_pixel = round(x *640)
                            # y_pixel = round(y* 480)
                            depth_image = ros_numpy.numpify(depth_image)
                            x_pixel = round((keypoint_[5,0] + keypoint_[6,0])/2)
                            y_pixel = round((keypoint_[5,1] + keypoint_[6,1])/2)
                            print(x_pixel, y_pixel)
                            center_square = depth_image[y_pixel][x_pixel]
                            print(center_square)
                            print(depth_image.shape)
                            center_square = center_square[~np.isnan(center_square)]
                            print(center_square)
                            Zinm = min(center_square)
                            Xinm = (x_pixel - self.principal_point_x) * (Zinm / self.focal_length_x)
                            Yinm = (y_pixel - self.principal_point_y) * (Zinm / self.focal_length_y)
                            if not Zinm is np.nan:
                                rospy.set_param('/person_location/x' , float(Xinm))
                                rospy.set_param('/person_location/y' , float(Yinm))
                                rospy.set_param('/person_location/z' , float(Zinm))
                                rospy.sleep(1)
                                rospy.set_param('/found_person' , True)
                            print('Gesture Detected')
                            # image_np = cv2.circle(image_np ,(index*20 , 0) , radius=10 , color = (self.color_r[index] , self.color_g[index] ,self.color_b[index])  , thickness = -1)
                    
                    
                    
                    # multi_message[ids[index]] = 0
                    # multi_message[ids[index]] += 1
        self.pub.publish(ros_numpy.msgify(Image , image_np , encoding = 'rgb8'))
        # for i in range(len(multi_message)):
            # image_np = cv2.circle(image_np , ())


                

if __name__== "__main__":
    rospy.init_node('yolo_trial')
    obj = yolo_trial()
    rospy.spin()