#!/usr/bin/env python
print("FLoofle")

import sys
import rospy
# from move_base_msgs.msg import MoveBaseAction , MoveBaseGoal
import moveit_commander
import actionlib
from pal_interaction_msgs.msg import TtsActionGoal, TtsAction, TtsGoal
from geometry_msgs.msg import PoseStamped
from play_motion_msgs.msg import PlayMotionActionGoal

print("after imports")

class arm_move:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.ac = actionlib.SimpleActionClient('/tts' , TtsAction)
        self.pub = rospy.Publisher('/play_motion/goal' , PlayMotionActionGoal , queue_size=1)
        self.ac.wait_for_server()
        self.tts_goal = TtsActionGoal()
        self.tts_goal.goal.rawtext.text = 'Hi,_I_am_up'
        self.tts_goal.goal.rawtext.lang_id = 'en_GB'
        self.ac.send_goal(self.tts_goal.goal)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.reference_frame = 'arm_1_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.allow_replanning(True)
        self.target_pose = PoseStamped()
        self.close_msg =  PlayMotionActionGoal()
        self.close_msg.goal.motion_name='close_gripper'
        self.open_msg =  PlayMotionActionGoal()
        self.open_msg.goal.motion_name='open_gripper'
        self.target_pose.header.frame_id = self.reference_frame
        self.object1_move_1 = [0.07 , 1.02 , -0.43 , 1.17 , -1.54 , 1.37 , 0.0]
        self.object1_move_2 = [0.82 , 0.9 , -0.59 , 1.36 , -1.93 , 1.19 , 0.0]
        self.object1_move_3 = [0.82 , 0.72 , -0.53 , 1.46 , -2.02 , 0.93 , 0.0]
        self.object2_move_1 = [0.07 , 1.02 , -0.43 , 1.17 , -1.54 , 1.37 , 0.0]
        self.object2_move_2 = [0.45 , 0.32 , -0.64 , 1.94 , 0.45 , 0.9 , 0.02]
        self.object2_move_3 = [0.76 , 0.34 , -0.6 , 1.63 , 0.54 , 0.73 , -0.03]
        self.object3_move_1 = [0.07 , 1.02 , -0.43 , 1.17 , -1.54 , 1.37 , 0.0]
        self.object3_move_2 = [2.1 , 1.02 , 0.62 , 1.63 , -1.24 , 0.47 , -0.49]
        self.object3_move_3 = [2.1 , 0.75 , 0.4 , 1.54 , -0.90 , 0.48 , -0.39]

        self.object_final = [0.53, 0.56, -1.19, 1.10, 0.58, -0.03, 0.04]
        self.object_home = [0.2, -1.34, -0.2, 1.94, -1.57, 1.37, 0.0]

    def goto_obj1(self):
        """
        A Function for reaching to object 1 # change the name of the object here
        """
        self.arm.go(self.object1_move_1 , wait=True)
        rospy.sleep(2)
        self.arm.go(self.object1_move_2 , wait=True)
        rospy.sleep(2)
        self.pub.publish(self.open_msg)
        rospy.sleep(2)
        self.arm.go(self.object1_move_3 , wait=True)
        self.pub.publish(self.close_msg)
        rospy.sleep(2)
        self.arm.go(self.object_final)
        rospy.sleep(2)
        self.pub.publish(self.open_msg)
        rospy.sleep(2)
        self.arm.go(self.object1_move_1)
        rospy.sleep(2)
        self.arm.go(self.object_home)
        self.pub.publish(self.close_msg)


    def goto_obj2(self):
        """
        A Function for reaching to object 1 # change the name of the object here
        """
        self.arm.go(self.object2_move_1 , wait=True)
        rospy.sleep(2)
        self.arm.go(self.object2_move_2 , wait=True)
        rospy.sleep(2)
        self.pub.publish(self.open_msg)
        rospy.sleep(2)
        self.arm.go(self.object2_move_3 , wait=True)
        self.pub.publish(self.close_msg)
        rospy.sleep(2)
        self.arm.go(self.object_final)
        rospy.sleep(2)
        self.pub.publish(self.open_msg)
        rospy.sleep(2)
        self.arm.go(self.object1_move_1)
        rospy.sleep(2)
        self.arm.go(self.object_home)
        self.pub.publish(self.close_msg)


    def goto_obj3(self):
        """
        A Function for reaching to object 1 # change the name of the object here
        """
        self.arm.go(self.object3_move_1 , wait=True)
        rospy.sleep(2)
        self.arm.go(self.object3_move_2 , wait=True)
        rospy.sleep(2)
        self.pub.publish(self.open_msg)
        rospy.sleep(2)
        self.arm.go(self.object3_move_3 , wait=True)
        self.pub.publish(self.close_msg)
        rospy.sleep(2)
        self.arm.go(self.object_final)
        rospy.sleep(2)
        self.pub.publish(self.open_msg)
        rospy.sleep(2)
        self.arm.go(self.object1_move_1)
        rospy.sleep(2)
        self.arm.go(self.object_home)
        self.pub.publish(self.close_msg)


    def speak(self, params):
        """
        A function to speaking 
        
        """
        self.tts_goal.goal.rawtext.text = params # 'Please_take_the_bottle_from_my_hand'
        self.ac.send_goal(self.tts_goal.goal  )

arm = arm_move()

print("After declaration")

def speak(tosay):
    """Function to make the tiago speak arbitrary words

    Args:
        tosay (string): A string to say using the speaker
    """
    arm.speak(tosay)

def goto_object_1():
    """Moves the arm to object 1, which is the tin of baked beans."""
    arm.goto_obj1()

def goto_object2():
    """
    Moves the arm to object 2, which is the can of pringles crisps."""
    arm.goto_obj2()

def goto_object3():
    """
    Moves the arm to object 3, which is the coffee cup."""
    arm.goto_obj3()
