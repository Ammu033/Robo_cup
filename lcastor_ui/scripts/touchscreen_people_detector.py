#!/usr/bin/env python

import rospy
import time
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray

poseArray_pub = None

def screen_touched_cb(msg):
    # print "Button pressed"
    if poseArray_pub is not None:
        for _ in range(20):

            pose_array = PoseArray()
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = "head_xtion_rgb_optical_frame"
            pose = Pose()
            pose.position.x = np.random.normal(0, 0.1, 1)[0]
            pose.position.y = np.random.normal(0.5, 0.05, 1)[0]
            pose.position.z = np.random.normal(0.8, 0.05, 1)[0]
            pose.orientation.y = 1.
            pose_array.poses.append(pose)

	    poseArray_pub.publish(pose_array)
            time.sleep(0.05)

if __name__ == "__main__":
    rospy.init_node("touchscreen_people_detector")

    rospy.Subscriber("/interface/buttonPressed", String, screen_touched_cb)

    poseArray_pub = rospy.Publisher("/touchscreen_people_detector/bounding_box_centres", PoseArray, queue_size=1)

    rospy.spin()
