#! /usr/bin/python3

import rospy
from robocup_human_sensing.msg import RegionOfInterest2D 
from darknet_ros_msgs.msg import BoundingBoxes

rospy.init_node('people_roi')
pub = rospy.Publisher('/people_roi' , RegionOfInterest2D  ,queue_size = 10)

def callback(data):
    msg = RegionOfInterest2D()
    for i in range(len(data.bounding_boxes)):
        if(data.bounding_boxes[i].Class == 'person'):
            msg.ids.append(i) 
            msg.x.append( data.bounding_boxes[i].xmin)
            msg.y.append( data.bounding_boxes[i].ymin)
            msg.w.append( data.bounding_boxes[i].xmax - data.bounding_boxes[i].xmin)
            msg.h.append( data.bounding_boxes[i].ymax - data.bounding_boxes[i].ymin)
            msg.c.append( data.bounding_boxes[i].probability)
    rospy.loginfo(msg)
    pub.publish(msg)

rospy.Subscriber('/darknet_ros/bounding_boxes' , BoundingBoxes , callback) 
rospy.spin()
