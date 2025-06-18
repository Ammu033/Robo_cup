#! /usr/bin/python3

import rospy
from robocup_human_sensing.msg import RegionOfInterest2D 
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import UInt64

DEPTH_IMAGE_TOPIC = '/xtion/depth/image_raw'
OBJECT_DETECTION_TOPIC = '/darknet_ros/bounding_boxes'
PERSON_CLASS = 'person'
rospy.init_node('people_roi')
pub = rospy.Publisher('/people_roi' , RegionOfInterest2D  ,queue_size = 10)
closed_person_pub = rospy.Publisher('/closestPersonDistance' ,UInt64 , queue_size = 10 )
cvb = CvBridge()
def callback(data):
    msg = RegionOfInterest2D()
    depth_data = rospy.wait_for_message( DEPTH_IMAGE_TOPIC , Image)
    depth_data_np = cvb.imgmsg_to_cv2(depth_data)
    people_depth_info = []
    for i in range(len(data.bounding_boxes)):
        if(data.bounding_boxes[i].Class == PERSON_CLASS):
            #msg.ids.append(i) 
            #msg.x.append( data.bounding_boxes[i].xmin)
            #msg.y.append( data.bounding_boxes[i].ymin)
            #msg.w.append( data.bounding_boxes[i].xmax - data.bounding_boxes[i].xmin)
            #msg.h.append( data.bounding_boxes[i].ymax - data.bounding_boxes[i].ymin)
            #msg.c.append( data.bounding_boxes[i].probability)
            # person_centroid = [int((data.bounding_boxes[i].ymin + data.bounding_boxes[i].ymax) / 2 ), int((data.bounding_boxes[i].xmin  + data.bounding_boxes[i].xmax)/2)]
            # rospy.loginfo(person_centroid)
            people_depth_info.append(
                depth_data_np[int((data.bounding_boxes[i].ymin + data.bounding_boxes[i].ymax) / 2 ), int((data.bounding_boxes[i].xmin  + data.bounding_boxes[i].xmax)/2)]
            )
    rospy.loginfo("d" + str(people_depth_info))
    if len(people_depth_info) > 0:
        min_index = people_depth_info.index(min(people_depth_info))
        closed_person_pub.publish(min(people_depth_info))
        msg.ids.append(min_index)
        msg.x.append( data.bounding_boxes[min_index].xmin)
        msg.y.append( data.bounding_boxes[min_index].ymin)
        msg.w.append( data.bounding_boxes[min_index].xmax - data.bounding_boxes[min_index].xmin)
        msg.h.append( data.bounding_boxes[min_index].ymax - data.bounding_boxes[min_index].ymin)
        msg.c.append( data.bounding_boxes[min_index].probability)
    
    # msg.ids.append()
    rospy.loginfo(msg)
    pub.publish(msg)

rospy.Subscriber( OBJECT_DETECTION_TOPIC, BoundingBoxes , callback) 
rospy.spin()
