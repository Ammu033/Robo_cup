#! /usr/bin/env python3

import rospy 
from std_msgs.msg import String

value=rospy.get_param("/string_topic/value") 
topic_name=rospy.get_param("/string_topic/topic_name") 
         
rospy.init_node('string_publish')	
pub_string = rospy.Publisher(topic_name, String, queue_size=1)
rospy.sleep(0.1)
pub_string.publish(value)


