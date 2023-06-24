#! /usr/bin/env python3

import rospy 
import time
from std_msgs.msg import String, Bool
import requests
from flask import Flask #, request, jsonify

app = Flask(__name__)
sender = "user"
rasa_endpoint = "http://localhost:5005/webhooks/rest/webhook"
# Setup ROS publisher
pub = rospy.Publisher('robot_speech', String, queue_size=1)
pub_trigger = rospy.Publisher("/nlp/trigger", Bool, queue_size=1)
pub_username = rospy.Publisher("/nlp/username", String, queue_size=1)
           
trigger=False
username=""

"""
@app.route("/webhook", methods=["POST"])
def rasa_action():
    rospy.logwarn("RASA ACTION")
    data = request.json
    person_name = data["tracker"]["slots"]["person_name"]
    rospy.logwarn("Got name: %s" % person_name)
    res = {"events": [], "responses": []}
    return jsonify(res)
@app.route("/webhook", methods=["POST"])
def rasa_action_go_to():
    data = request.json
    destination = data["tracker"]["slots"]["location"]
    rospy.logwarn("Got destination: %s" % destination)
    #res = {"events": [], "responses": [{"text": "Going now to the %s" % destination}]}
    res = {"events": [], "responses": []}
    return jsonify(res)
"""
def send_to_rasa(msg):
    global trigger, username
    text = msg
    rospy.loginfo('Heard message: "%s"  -- sending it to Rasa...' % text)

    results = requests.post(rasa_endpoint, json={"sender": sender, "message": text}).json()
    for r in results:
        #print(r)
        msg = String()
        msg.data = r["text"]
        pub.publish(msg)
        """
        if r["text"]=="Following you!" and trigger==False:
            trigger=True   
            pub_trigger.publish(trigger)
        elif r["text"]=="I have stopped!" and trigger==True:
            trigger=False
        pub_trigger.publish(trigger)
        #if username!=r["tracker"]["slots"]["person_name"] and r["tracker"]["slots"]["person_name"]!=None:
        #    username=r["tracker"]["slots"]["person_name"]
        #pub_username.publish(username)
        """

def subscribe_to_speech(msg):
    header_phrase=["tiago","thiago","theago","khiago","robot","diego"]
    for i in header_phrase:
        if i in msg.data.lower():
            send_to_rasa(msg.data)
            break
         
if __name__ == '__main__':
    time_init=time.time()  
    # Initialize our node       
    rospy.init_node('rasa_bridge')
    # Setup ROS subscription
    rospy.Subscriber('user_speech',String,subscribe_to_speech)
    #use same port as default RASA action server
    #app.run(host="0.0.0.0", port=5055)
    rospy.spin()
        
        
    


