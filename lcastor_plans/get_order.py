import os 
import sys
import rospy
import pnp_cmd_ros
from std_msgs.msg import String
from ollamamessages.msg import WhisperListening
import json
from pnp_cmd_ros import *
import time


def get_order(p , from_guest = False, to_barman = False , to_guest = False , list_of_items  =[]):
    if from_guest :
        publisher = rospy.Publisher('/planner_intention', String, queue_size=10, latch=True)
        publisher.publish('food_items')

        listening_pub = rospy.Publisher("/stt/listening", WhisperListening, queue_size = 1)

        p.exec_action('speak' , "Hi,_welcome_to_the_restaurant")
        # rospy.sleep(1.0)
        p.exec_action('speak' , "What_would_you_like_to_order?")
        rospy.sleep(0.5)
        listening_pub.publish(listening = True)
        ## IMPLEMENT AND ADD OLLAMA FOR GETTING LIST OF ORDER
        rospy.sleep(5.0)
        data_from_ollama = rospy.wait_for_message('/ollama_output' , String) #None # NEED TO FIX WITH OLLAMA FUNCTION
        print(data_from_ollama)
        list_of_items = json.loads(data_from_ollama.data)
        if len(list_of_items )> 1:
            list_of_items.insert(-1 , 'and') 
            list_of_items = ',_'.join(list_of_items)
        elif len(list_of_items) == 1 :
            list_of_items = list_of_items[0]
        else : 
            rospy.logerr('Length of List of Items is **ZERO**')
        print(list_of_items)
        p.exec_action('speak' , 'I_will_get_you_'+list_of_items)
        return list_of_items
    if to_barman : 
        p.exec_action('speak' , "Hi,_a_customer_ordered")
        # if len(list_of_items )> 1:
        #     list_of_items.insert(-1 , 'and') 
        #     list_of_items = ',_'.join(list_of_items)
        # elif len(list_of_items) == 1 :
        #     list_of_items = list_of_items[0]
        # else : 
        #     rospy.logerr('Length of List of Items is **ZERO**')
        # print(list_of_items)
        p.exec_action('speak' , list_of_items)
        p.exec_action('speak' , 'Could_you_please_place_the_ordered_items_on_the_tray_within_10_seconds._Thanks')
        rospy.sleep(10)
    if to_guest : 
        rospy.sleep(10)
        p.exec_action('speak' , "Hi,_Please_take_your_order_from_the_tray_within_10_seconds")
        rospy.sleep(10)
        p.exec_action('speak' , "Enjoy_your_meal")