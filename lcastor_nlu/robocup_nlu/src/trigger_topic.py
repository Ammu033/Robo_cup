#! /usr/bin/env python3

import rospy 
from std_msgs.msg import Bool

trigger=rospy.get_param("/trigger_topic/trigger") 
         
if __name__ == '__main__':
    # Initialize our node       
    rospy.init_node('trigger_publish')	
    #pub_trigger = rospy.Publisher("/nlp/trigger", Bool, queue_size=1)
    pub_trigger = rospy.Publisher("/nlp/trigger", Bool, queue_size=1)
    #priority_msg = Bool()
    rate = rospy.Rate(1/0.01)  # main loop frecuency in Hz
    #username="hola"
    print("TRIGGER",trigger)
    while not rospy.is_shutdown():	
        #pub_trigger.publish(robot.trigger)
        pub_trigger.publish(trigger)
        rate.sleep() #to keep fixed the publishing loop rate
    


