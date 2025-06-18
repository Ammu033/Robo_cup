#!/usr/bin/env python
import os
import sys
import rospy
import time
import subprocess

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + "/scripts")
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import pnp_cmd_ros
from pnp_cmd_ros import *

def wait_for_room_name():
    """Wait for Rasa to extract room name from user speech"""
    rospy.loginfo("Waiting for room name from Rasa...")
    
    # Reset the parameter
    rospy.set_param('/room_to_save', '')
    
    # Wait for room name to be set by Rasa
    start_time = rospy.get_time()
    room_name = ""
    
    while not room_name and not rospy.is_shutdown():
        if rospy.get_time() - start_time > 30.0:  # 30 second timeout
            rospy.logwarn("Timeout waiting for room name")
            return None
            
        room_name = rospy.get_param('/room_to_save', '').strip()
        time.sleep(0.5)
    
    return room_name if room_name else None

def call_save_pose(room_name):
    """Call the existing save_pose.py script with room name"""
    try:
        rospy.loginfo(f"Calling save_pose.py for room: {room_name}")
        
        rosrun_command = ['rosrun', 'tiagoauto', 'save_pose.py']
        # Create a process to run save_pose.py
        process = subprocess.Popen(rosrun_command, 
                                 stdin=subprocess.PIPE, 
                                 stdout=subprocess.PIPE, 
                                 stderr=subprocess.PIPE,
                                 text=True)
        
        # Send room name to save_pose.py input
        stdout, stderr = process.communicate(input=room_name + '\n')
        
        if process.returncode == 0:
            rospy.loginfo(f"✅ save_pose.py completed successfully")
            rospy.loginfo(f"Output: {stdout}")
            return True
        else:
            rospy.logerr(f"❌ save_pose.py failed: {stderr}")
            return False
            
    except Exception as e:
        rospy.logerr(f"❌ Error calling save_pose.py: {e}")
        return False

def start_voice_save_session(p):
    """Start voice-controlled pose saving"""
    try:
        # Activate Rasa to listen for room save intent
        rospy.loginfo("🎤 Activating voice recognition...")
        p.exec_action('speak', 'Ready_to_save_pose._Please_say_save_pose_as_room_name.')
        p.exec_action('activateRasa', 'room_name')
        
        # Wait for Rasa to extract room name
        room_name = wait_for_room_name()
        
        if room_name:
            rospy.loginfo(f"🎯 Room name received: '{room_name}'")
            
            # Call your existing save_pose.py
            if call_save_pose(room_name):
                p.exec_action('speak', f'Pose_saved_successfully_for_{room_name.replace(" ", "_")}')
                return True
            else:
                p.exec_action('speak', 'Failed_to_save_pose._Please_try_again.')
                return False
        else:
            p.exec_action('speak', 'No_room_name_received._Please_try_again.')
            return False
            
    except Exception as e:
        rospy.logerr(f"❌ Error in voice save session: {e}")
        p.exec_action('speak', 'Error_occurred._Please_try_again.')
        return False

if __name__ == "__main__":
    p = PNPCmd()
    p.begin()

    rospy.loginfo("🎯 Starting voice save session...")
    success = start_voice_save_session(p)
    
    if success:
        rospy.loginfo("✅ Voice save completed!")
    else:
        rospy.loginfo("❌ Voice save failed!")

    p.end()