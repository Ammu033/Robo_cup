#! /usr/bin/env python3

import rospy 
from std_msgs.msg import String

username=rospy.get_param("/username_topic/username") 
         
if __name__ == '__main__':
    # Initialize our node       
    rospy.init_node('username_publish')	
    #pub_trigger = rospy.Publisher("/nlp/trigger", Bool, queue_size=1)
    pub_username = rospy.Publisher("/nlp/username", String, queue_size=1)
    #priority_msg = Bool()
    rate = rospy.Rate(1/0.01)  # main loop frecuency in Hz
    #username="hola"
    print("USERNAME",username)
    while not rospy.is_shutdown():	
        #pub_trigger.publish(robot.trigger)
        pub_username.publish(username)
        rate.sleep() #to keep fixed the publishing loop rate
    


