#!/usr/bin/python
import rospy
import sys
import tf
import moveit_commander
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, PoseStamped

class tiago_moveit:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.move_group = moveit_commander.MoveGroupCommander("arm_torso")
        self.move_group.set_end_effector_link("gripper_grasping_frame")
        self.move_group.set_goal_tolerance(0.05)

        rospy.Subscriber("/click_pose", PoseStamped, self.callback, queue_size=1)

        self.translation = [0, 0, 0]
        self.rotation = [0, 0, 0, 0]

    def click_pose(self):
        pose = Pose()

        pose.position.x = self.translation[0]
        pose.position.y = self.translation[1]
        pose.position.z = self.translation[2]

        pose.orientation.x = self.rotation[0]
        pose.orientation.y = self.rotation[1]
        pose.orientation.z = self.rotation[2]
        pose.orientation.w = self.rotation[3]
        
        return pose

    def in_front_position(self):

        pose = self.click_pose()

        pose.position.x = self.translation[0] - 0.05
        
        return pose

    def callback(self, data):
        self.translate_pose_to_base(data)
        
        waypoints = [self.click_pose()]
        
        (plan, fraction) = self.compute_waypoint_path(waypoints)

        self.execute_plan(plan, fraction)


    def translate_pose_to_base(self, pose_stamped):
        pose_to_base = self.listener.transformPose("/base_footprint", pose_stamped)

        trans = pose_to_base.pose.position
        rot = pose_to_base.pose.orientation

        self.trans = [trans.x, trans.y, trans.z]
        self.rot = [rot.x, rot.y, rot.z, rot.w]

    def display_startup_info(self, print_robot_state=False):
        planning_frame = self.move_group.get_planning_frame()
        print("Planning Frame: %s" % planning_frame)

        eef_link = self.move_group.get_end_effector_link()
        print("End effector link: %s" % eef_link)

        group_names = self.robot.get_group_names()
        print("Available Planning Groups: %s" % group_names)

        if (print_robot_state):
            print("=== Printing Robot State ===")
            print(self.robot.get_current_state())
            print("=== Stopped Printing Robot State ===")

    def compute_waypoint_path(self, waypoints):
        print("Planning...")
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 1.0, 0.2, avoid_collisions=False)
        print("Found plan!")
        print("fraction: %s" % fraction)
        return plan, fraction

    def execute_plan(self, plan, fraction):
        if fraction == 1.0:
            self.move_group.execute(plan, wait=True)

            self.move_group.stop()

            self.move_group.clear_pose_targets()
        else:
            print("Fraction was not 1.0, not executing path")

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("tiago_moveit_node")
    robot_controller = tiago_moveit()
    robot_controller.display_startup_info()
    
    rospy.spin()

if __name__ == "__main__":
    main()