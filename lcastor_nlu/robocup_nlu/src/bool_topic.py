#! /usr/bin/env python3

import rospy 
from std_msgs.msg import Bool

value=rospy.get_param("/bool_topic/value") 
topic_name=rospy.get_param("/bool_topic/topic_name") 
              
rospy.init_node('bool_publish')	
pub_bool = rospy.Publisher(topic_name, Bool, queue_size=1)
rospy.sleep(0.1)
pub_bool.publish(value)
   