#!/usr/bin/python
import rospy
import sys
import tf
import moveit_commander
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
import threading
import tf2_ros
import tf2_geometry_msgs #import the packages first
import tf_conversions
import geometry_msgs.msg
from tf.transformations import quaternion_matrix #Return homogeneous rotation matrix from quaternion.
from tf.transformations import quaternion_from_matrix  #Return quaternion from rotation matrix.
from tf.transformations import quaternion_multiply
from std_msgs.msg import Bool
import numpy as np
import copy

recieved_poses = PoseStamped()
target_poses = geometry_msgs.msg.PoseArray()
pose_transformed = PoseStamped()

import math

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):

    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class tiago_moveit:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.move_group = moveit_commander.MoveGroupCommander("arm_torso")
        self.move_group.set_end_effector_link("gripper_grasping_frame")
        self.move_group.set_goal_tolerance(0.05)
        self.pose_grasp = Pose()
        self.go_graspn_pose = Bool()
        dir(self.go_graspn_pose)
        self.go_graspn_pose.data=False
        rospy.Subscriber('/grasp_points', PoseArray, self.callbackGraspPoints)

        rospy.Subscriber("/click_pose", PoseStamped, self.callback, queue_size=1)
        
        self.recieved_poses = PoseArray()
        self.translation = [0, 0, 0]
        self.rotation = [0, 0, 0, 0]

    def grasp_position_listener(self):
        rospy.Subscriber('/grasp_points', PoseArray, self.callbackGraspPoints)
        rospy.spin()

    # Helpers for rotation....
    def create_rotZ( self, angle_rads  ):
        c, s = np.cos(angle_rads), np.sin(angle_rads)
        R = np.array( ( (c, -s, 0), (s, c, 0) , (0,0,1) ) )
        return R

    def create_rotY( self, angle_rads  ):
        c, s = np.cos(angle_rads), np.sin(angle_rads)
        R = np.array( ( (c, 0, s), (0, 1, 0) , (-s, 0, c) ) )
        return R

    def create_rotX( self, angle_rads  ):
        c, s = np.cos(angle_rads), np.sin(angle_rads)
        R = np.array( ( (1, 0, 0), (0, c, -s) , (0, s, c) ) )
        return R

    def callbackGraspPoints(self, msg):
        global recieved_poses, pose_transformed
        move_group = self.move_group

        R_0 = np.eye(4,4)
        R_0[ 0:3,0:3 ] = np.matmul( self.create_rotX( 180 / 180. * np.pi ) , self.create_rotY( -90 / 180. * np.pi ) ,  self.create_rotZ( 00 / 180. * np.pi  ) )
        quaternion_of_R = quaternion_from_matrix( R_0 )

        br = tf2_ros.TransformBroadcaster()
        target_poses.header.frame_id = 'base_footprint'
        target_poses.header.stamp = rospy.Time().now()
        point_size =len(msg.poses)

        for i in range(point_size):
            recieved_poses = msg.poses[i]
            t = geometry_msgs.msg.PoseStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "xtion_rgb_optical_frame"
            t.pose.position.x = recieved_poses.position.x
            t.pose.position.y = recieved_poses.position.y
            t.pose.position.z = recieved_poses.position.z
            t.pose.orientation.x = recieved_poses.orientation.x
            t.pose.orientation.y = recieved_poses.orientation.y
            t.pose.orientation.z = recieved_poses.orientation.z
            t.pose.orientation.w = recieved_poses.orientation.w

            tf_buffer = tf2_ros.Buffer(rospy.Duration(5000.0)) #tf buffer length
            tf_listener = tf2_ros.TransformListener(tf_buffer)
            #trans = tf_buffer.lookup_transform("base_footprint", 'xtion_rgb_frame', rospy.Time())

            trans = tf_buffer.lookup_transform('base_footprint',
                                            'xtion_rgb_optical_frame', #source frame
                                            rospy.Time.now()
                                            , #get the tf at first available time
                                            rospy.Duration(3.0))
            pose_transformed = tf2_geometry_msgs.do_transform_pose(t, trans)
            quaternion_of_wp = [ pose_transformed.pose.orientation.x, pose_transformed.pose.orientation.y, pose_transformed.pose.orientation.z, pose_transformed.pose.orientation.w ]
            rslt_quaternion = quaternion_multiply( quaternion_of_wp, quaternion_of_R )
            # set the resulting orientation to marker ( original_orientation (+) pan_of_ptz )
            pose_transformed.pose.orientation.x = rslt_quaternion[0]
            pose_transformed.pose.orientation.y = rslt_quaternion[1]
            pose_transformed.pose.orientation.z = rslt_quaternion[2]
            pose_transformed.pose.orientation.w = rslt_quaternion[3]
            #pose_grasp = Pose()
            self.pose_grasp.position.x = pose_transformed.pose.position.x-0.4
            self.pose_grasp.position.y = pose_transformed.pose.position.y#+0.5
            self.pose_grasp.position.z = pose_transformed.pose.position.z+0.3
            self.pose_grasp.orientation.x = pose_transformed.pose.orientation.x
            self.pose_grasp.orientation.y = pose_transformed.pose.orientation.y
            self.pose_grasp.orientation.z = pose_transformed.pose.orientation.z
            self.pose_grasp.orientation.w = pose_transformed.pose.orientation.w
            
            target_poses.poses.append(self.pose_grasp)

            tt = geometry_msgs.msg.TransformStamped()

            tt.header.stamp = rospy.Time.now()
            tt.header.frame_id = "base_footprint"
            tt.child_frame_id = "test_"+str(i)
            tt.transform.translation.x = pose_transformed.pose.position.x-0.4
            tt.transform.translation.y = pose_transformed.pose.position.y#-0.5
            tt.transform.translation.z = pose_transformed.pose.position.z+0.3
            tt.transform.rotation.x = pose_transformed.pose.orientation.x
            tt.transform.rotation.y = pose_transformed.pose.orientation.y
            tt.transform.rotation.z = pose_transformed.pose.orientation.z
            tt.transform.rotation.w = pose_transformed.pose.orientation.w
            

            br.sendTransform(tt)
            self.go_graspn_pose.data = True

            # waypoints = []
            # waypoints.append(copy.deepcopy(pose_grasp))

            # (plan, fraction) = move_group.compute_cartesian_path(
            #     waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
            # )  # jump_threshold
            # self.execute_plan(plan)
            self.go_to_pose_goal(self.pose_grasp)
            #print("pose_transformed",recieved_poses)

           # print('Quarternion', pose_transformed)


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

    def go_to_pose_goal(self,pose_goal):
        move_group = self.move_group
        move_group.set_pose_target(pose_goal)
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()


        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
    
    def go_to_pose_goal2(self):
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.6
        pose_goal.position.y = 0.2
        pose_goal.position.z = 0.8
        move_group.set_pose_target(pose_goal)
        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()


    def compute_waypoint_path(self, waypoints):
        print("Planning...")
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 1.0, 0.2, avoid_collisions=False)
        print("Found plan!")
        print("fraction: %s" % fraction)
        return plan, fraction
    
    def execute_plan(self, plan):
        global grasp
        move_group = self.move_group
        #print(grasp)
        #print(self.z)

        move_group.execute(plan, wait=True)
    # def execute_plan(self, plan, fraction):
    #     if fraction == 1.0:
    #         self.move_group.execute(plan, wait=True)

    #         self.move_group.stop()

    #         self.move_group.clear_pose_targets()
    #     else:
    #         print("Fraction was not 1.0, not executing path")

    def graspn_pick(self):

        if self.go_graspn_pose.data == True:
            self.go_to_pose_goal(self.pose_grasp)
            self.go_graspn_pose.data = False
        else:
            a=1
            #print("No grasp candidate")


def main():
    global recieved_poses, pose_transformed
    moveit_commander.roscpp_initialize(sys.argv)
    print("1")

    rospy.init_node("tiago_moveit_node")
    robot_controller = tiago_moveit()

    #robot_controller.display_startup_info()
    rate = rospy.Rate(1000)

    #robot_controller .go_to_pose_goal2()  # uncomment to move specific pose
    #print("moved in task space")

    while not rospy.is_shutdown():
        #robot_controller.graspn_pick()  

        rate.sleep()

        #rospy.spin()

if __name__ == "__main__":
    main()