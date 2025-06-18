#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Nikolaus Wagner (C) 2023
# nwagner@lincoln.ac.uk

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from pathlib import Path

from lcastor_perception.srv import DetectObjects

from sensor_msgs.msg import Image  
import ros_numpy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from scipy.spatial.transform import Rotation as R

# Define global variabile 
global pub_target_rel_pose
global pub_target_rel_pose_stamped

def findObject(image):

  global pub_target_rel_pose
  global pub_target_rel_pose_stamped

  print("omer1")

  bridge = CvBridge()
  print("omer1")
  img_path = "/home/omer/tiago_public_ws/src/lcastor_perception/test/test_imgs/"

  labels = np.genfromtxt("/home/omer/tiago_public_ws/src/lcastor_perception/src/lcastor_perception/labels_coco.txt", dtype=str)
  #labels = np.genfromtxt("../src/lcastor_perception/labels_imagenet.txt", dtype=str)
  print("omer2")

  for child in Path(img_path).iterdir():
    if child.is_file():
      print(img_path + child.name)
      img = cv2.imread(img_path + child.name)
      img_msg = bridge.cv2_to_imgmsg(img, encoding="passthrough")

      detect_objects = rospy.ServiceProxy("detect_objects", DetectObjects)
      results = detect_objects(img_msg)
      for i, detection in enumerate(results.detections.detections):
        print("omer33")

        if detection.results[0].score > 0.8:
          print(detection.results[0].score, labels[detection.results[0].id - 1])
          x_0 = int(detection.bbox.center.x - detection.bbox.size_x / 2)
          x_1 = int(detection.bbox.center.x + detection.bbox.size_x / 2)
          y_0 = int(detection.bbox.center.y - detection.bbox.size_y / 2)
          y_1 = int(detection.bbox.center.y + detection.bbox.size_y / 2)
          print("omer333",detection.bbox.center)

          img = cv2.rectangle(img, (x_0, y_0), (x_1, y_1), color=(255, 0, 0))
          img = cv2.putText(img, labels[detection.results[0].id - 1], (x_0, y_0),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0))

      segmask = bridge.imgmsg_to_cv2(results.detections.detections[0].source_img, desired_encoding="passthrough")
      print("omer41")

      cv2.imshow("segmask", segmask * 255 * 10)
      print("omer42")

      cv2.imshow("img", img)
      cv2.waitKey()
      print()
      print("omer43")
  
      # a representation of pose in free space, composed of position & orientation. 
    messageTargetPose = Pose()
    # if object detected substitute coordinate use result to do translation 
    # translation coordinate (z,x,y)[meter]
    p = results.detections.detections[0].translation
    print(-p[2], p[0], p[1])

    # if object detected substitute coordinate use result to do rotation as quaternions
    # *** Rotation in 3 dimensions can be represented using unit norm quaternions***.
    # quaternion coordinate (x,y,z,w) [rad]
    q = R.from_matrix(results.detections.detections[0].rotation).as_quat()

    # find orientation via quaternions
    messageTargetPose.orientation = Quaternion(q[0], q[1], q[2], q[3])
    # find position via transformation 
    messageTargetPose.position = Point(-p[2], p[0], p[1])
    
    # publish messageTargetPose on target relative position 
    pub_target_rel_pose.publish(messageTargetPose)
    
    # substitute in pose_stamped, a Pose with reference coordinate frame & timestamp
    pose_stamped = PoseStamped(pose = messageTargetPose)
    # pose_stamped use as frame_id --> frame of rgd camera 
    pose_stamped.header.frame_id='xtion_rgb_frame'

    # publish relative postion of object on pose_stamped 
    pub_target_rel_pose_stamped.publish(pose_stamped)


if __name__ == '__main__':

    # ros node initialization --> Find Object 
    rospy.init_node('FindObject')

    # subscribe to rgb camera to take image information
    sub_camera = rospy.Subscriber('/xtion/rgb/image_raw', Image, findObject)
    #sub_camera = rospy.Subscriber('/zed2i/zed_node/rgb/image_rect_color', Image, findObject)

    # publishe to traget_pose/relative positione of object detected 
    pub_target_rel_pose = rospy.Publisher(
        '/lcastor/target_pose/relative', Pose, queue_size=1)

    # publishe to traget_pose/relative/stamped to print positione of object detected 
    pub_target_rel_pose_stamped = rospy.Publisher(
        '/lcastor/target_pose/relative/stamped', PoseStamped, queue_size=1)   

    # start infinite loop until it receivces a shutdown  signal (Ctrl+C)
    rospy.spin()
