#!/usr/bin/env python

import rospy
import yaml
import os
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def listen_for_room_name():
    text = input("please provide the name of location: ")
    try:
        print(f"🔊 You said: {text}")
        return text
    except ValueError:
        print("❌ Could not understand audio.")
    
    return None

def map_text_to_room_key(text):
    text = text.replace(" ", "")
    if "room1" in text or "roomone" in text:
        return "room_1"
    elif "room2" in text or "roomtwo" in text:
        return "room_2"
    else:
        return None

def load_pose(room_name):
    yaml_path = os.path.expanduser("~/ros_ws/src/LCASTOR/goal/goal.yaml")
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    return data.get(room_name, None)

def send_navigation_goal(pose):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = pose["x"]
    goal.target_pose.pose.position.y = pose["y"]
    goal.target_pose.pose.position.z = pose["z"]
    goal.target_pose.pose.orientation.x = pose["qx"]
    goal.target_pose.pose.orientation.y = pose["qy"]
    goal.target_pose.pose.orientation.z = pose["qz"]
    goal.target_pose.pose.orientation.w = pose["qw"]

    rospy.loginfo("🔁 Sending goal...")
    client.send_goal(goal)
    client.wait_for_result()

    if client.get_result():
        rospy.loginfo("✅ Reached destination!")
    else:
        rospy.logwarn("⚠️ Failed to reach destination.")

def main():
    rospy.init_node('voice_to_nav')

    text = listen_for_room_name()
    if not text:
        return

    room_key = map_text_to_room_key(text)
    if not room_key:
        rospy.logerr("❌ Couldn't match voice input to any known room.")
        return

    pose = load_pose(room_key)
    if not pose:
        rospy.logerr("❌ Room not found in YAML file.")
        return

    send_navigation_goal(pose)

if __name__ == "__main__":
    main()
