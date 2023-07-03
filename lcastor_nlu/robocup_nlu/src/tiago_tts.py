#! /usr/bin/env python3

import rospy 
import time
from std_msgs.msg import String
from pal_interaction_msgs.msg import TtsActionGoal, TtsAction
import actionlib


def subscribe_to_speech(msg):
    speak(msg.data)
    
def speak(msg):
    ac = actionlib.SimpleActionClient("/tts", TtsAction)
    ac.wait_for_server()

    # create goal
    goal = TtsActionGoal()
    goal.goal.rawtext.text = msg
    goal.goal.rawtext.lang_id = "en_GB"

    # send goal
    ac.send_goal(goal.goal)
    ac.wait_for_result()
         
if __name__ == '__main__':
    time_init=time.time()  
    # Initialize our node       
    rospy.init_node('tiago_tts')
    # Setup ROS subscription
    rospy.Subscriber('robot_speech',String,subscribe_to_speech)
    rospy.spin()
        
        
    


