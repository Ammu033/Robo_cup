#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def main():
    rospy.init_node('go_to_saved_pose')

    # Create SimpleActionClient
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Fill in your saved AMCL pose here:
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # Always "map" for global nav
    goal.target_pose.header.stamp = rospy.Time.now()

    # Inserted real /amcl_pose values
    goal.target_pose.pose.position.x = 3.5621731402518204
    goal.target_pose.pose.position.y = -4.671101007945189
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = -0.882735795603073
    goal.target_pose.pose.orientation.w = 0.4698696789121529

    rospy.loginfo("Sending goal to move_base...")
    client.send_goal(goal)
    client.wait_for_result()

    if client.get_result():
        rospy.loginfo("Reached the goal!")
    else:
        rospy.logwarn("Failed to reach the goal.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
