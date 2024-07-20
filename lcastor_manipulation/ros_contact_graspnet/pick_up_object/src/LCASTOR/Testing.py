#!/usr/bin/env python

import rospy

from pick_up_object.srv import DetectObjects
from geometric_grasp.utils import convertCloudFromRosToOpen3d
from geometry_msgs.msg import PoseArray,Pose,PoseStamped
from spherical_grasps_server import SphericalGrasps
import tf2_ros
import tf2_geometry_msgs
import moveit_commander
import sys
from actionlib import SimpleActionClient
from moveit_msgs.msg import PickupAction, PickupGoal
from time import sleep
import tf.transformations as tf

from lcastor_grasping.utils.detectObjs import detectObjs
from lcastor_grasping.utils.moveObj import moveObj

# def detect_objs():
#     """
#         Calls detection server (Mask-RCNN)

#         Return:
#             detection {DetectObjects}
#     """

#     print('waiting for detect_objects')
#     rospy.wait_for_service('detect_objects', timeout=10)
#     try:
#         detect = rospy.ServiceProxy('detect_objects', DetectObjects)
#         resp = detect()
#         print('detection done!')
#         return resp
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)

# def transform_pose(pose_stamped, target_frame):
#     """
#     Transforms a PoseStamped from its current frame to the target frame.

#     :param pose_stamped: PoseStamped to be transformed.
#     :param target_frame: Target frame to transform the pose to.
#     :return: Transformed PoseStamped.
#     """
#     # Initialize the transform buffer and listener
#     tf_buffer = tf2_ros.Buffer()
#     tf_listener = tf2_ros.TransformListener(tf_buffer)
#     sleep(1)

#     try:
#         # Lookup the transform from the pose's frame to the target frame
#         transform = tf_buffer.lookup_transform(target_frame, pose_stamped.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

#         # Transform the pose
#         transformed_pose_stamped = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

#         transformed_pose_stamped.pose.orientation.x = 0
#         transformed_pose_stamped.pose.orientation.y = 0
#         transformed_pose_stamped.pose.orientation.z = 0
#         transformed_pose_stamped.pose.orientation.w = 1

#         return transformed_pose_stamped
#     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
#         rospy.logwarn("Transform error: %s", e)
#         return None

# class getGrasps():
#     def __init__(self, method="Spherical"):
#         if method == "Spherical":
#             self.sg = SphericalGrasps()
#             self.getGrasps = self.getSphericalGrasps
#         else:
#             print("Method Not Recognised")
          
#     def getSphericalGrasps(self, object_pose, approach = "top", publish=False):
        
#         if approach == "front":            
#             object_pose.pose.position.x = object_pose.pose.position.x #+ 0.065 # depth offset for gripping above
#             object_pose.pose.position.y = object_pose.pose.position.y - 0.03 # left/right offset
#             object_pose.pose.position.z = object_pose.pose.position.z #+ 0.1 # offset for gripping above

#         elif approach == "top":
#             object_pose.pose.position.x = object_pose.pose.position.x #+ 0.065 # depth offset for gripping above
#             object_pose.pose.position.y = object_pose.pose.position.y #- 0.03 # left/right offset
#             object_pose.pose.position.z = object_pose.pose.position.z + 0.1 # offset for gripping above


#         if publish is True:
#             possible_grasps = self.sg.create_grasps_from_object_pose(object_pose)
#         else:
#             possible_grasps = self.sg.generate_grasp_poses(object_pose)

#         if approach == "front":   
#             possible_grasps = self.rotate_grasp_pose(possible_grasps)
#             possible_grasps = possible_grasps[:10]
#         elif approach == "top": 
#             possible_grasps.reverse()
#             possible_grasps = possible_grasps[:10]


#         return possible_grasps
    
#     def rotate_grasp_pose(self, grasps, angle=90):
#         rotated_grasps = []
#         for grasp in grasps:
#             # Define the rotation quaternion for a 90-degree rotation around the x-axis
#             angle_rad = angle * (3.141592653589793 / 180.0)
#             rotation_quaternion = tf.quaternion_from_euler(angle_rad, 0, 0)

#             # Current orientation quaternion
#             current_orientation = [
#                 grasp.grasp_pose.pose.orientation.x,
#                 grasp.grasp_pose.pose.orientation.y,
#                 grasp.grasp_pose.pose.orientation.z,
#                 grasp.grasp_pose.pose.orientation.w
#             ]

#             # Perform the quaternion multiplication
#             new_orientation = tf.quaternion_multiply(rotation_quaternion, current_orientation)

#             # Update the grasp_pose with the new orientation
#             grasp.grasp_pose.pose.orientation.x = new_orientation[0]
#             grasp.grasp_pose.pose.orientation.y = new_orientation[1]
#             grasp.grasp_pose.pose.orientation.z = new_orientation[2]
#             grasp.grasp_pose.pose.orientation.w = new_orientation[3]

#             rotated_grasps.append(grasp)
#         return rotated_grasps
    
#     def changeMethod(self,method):
#         if method == "Spherical":
#             self.sg = SphericalGrasps()
#             self.getGrasps = self.getSphericalGrasps
#             return True
#         else:
#             return False

# class robotControlPlanner():
#     def __init__(self):
#         # Initialize the move_group API
#         moveit_commander.roscpp_initialize(sys.argv)

#         # Initialize the RobotCommander object which provides information such as the robot's kinematic model and the robot's current joint states
#         self.robot = moveit_commander.RobotCommander()

#         # Initialize the PlanningSceneInterface object which provides a remote interface for getting, setting, and updating the robot's internal understanding of the surrounding world:
#         self.scene = moveit_commander.PlanningSceneInterface()

#         # Initialize the MoveGroupCommander object. This object is an interface to one group of joints. In this case, we are using the group of joints in the Tiago's arm.
#         group_name = "arm_torso"  # Replace with your MoveIt group name
#         self.group = moveit_commander.MoveGroupCommander(group_name)

#         # Allow replanning to increase the odds of a solution
#         self.group.allow_replanning(True)

#         # Set the goal pose tolerance
#         self.group.set_goal_position_tolerance(0.1)
#         self.group.set_goal_orientation_tolerance(0.1)

#         self.pickup_ac = SimpleActionClient('/pickup', PickupAction)
#         self.pickup_ac.wait_for_server()
#         self.links_to_allow_contact = rospy.get_param('~links_to_allow_contact', None)
#         if self.links_to_allow_contact is None:
#             rospy.logwarn("Didn't find any links to allow contacts... at param ~links_to_allow_contact")
#         else:
#             rospy.loginfo("Found links to allow contacts: " + str(self.links_to_allow_contact))

#     def add_objects_to_scene(self,objects):
#         return False

#     def grasp_object(self,object_pose, grasp_pose):
#         goal = self.createPickupGoal(
# 			"arm_torso", "part", object_pose, grasp_pose, self.links_to_allow_contact)
#         self.pickup_ac.send_goal(goal)
#         result = self.pickup_ac.wait_for_result()
#         print(result)
#         return result


#     def createPickupGoal(self, group="arm_torso", target="part",
# 					 grasp_pose=PoseStamped(),
# 					 possible_grasps=[],
# 					 links_to_allow_contact=None):
#         """ Create a PickupGoal with the provided data"""
#         pug = PickupGoal()
#         pug.target_name = target
#         pug.group_name = group
#         pug.possible_grasps.extend(possible_grasps)
#         pug.allowed_planning_time = 35.0
#         pug.planning_options.planning_scene_diff.is_diff = True
#         pug.planning_options.planning_scene_diff.robot_state.is_diff = True
#         pug.planning_options.plan_only = False
#         pug.planning_options.replan = True
#         pug.planning_options.replan_attempts = 30
#         pug.allowed_touch_objects = []
#         pug.attached_object_touch_links = ['<octomap>']
#         #pug.attached_object_touch_links.extend(links_to_allow_contact)

#         return pug



def main():
    # Initialize the ROS node with a name "basic_node"
    rospy.init_node('testing_grasping', anonymous=True)

    # Log a message to indicate that the node has started
    rospy.loginfo("Basic ROS Grasping Node has been started")

    # Register Publishers
    pub_pose_array = rospy.Publisher('/object_poses', PoseArray, queue_size=10)
    pub_pose = rospy.Publisher('/object_pose', PoseStamped, queue_size=10)

    pickerUpper = moveObj(method="ContactGraspnet")

    # Detect Objects
    rcnn_detector = detectObjs("rcnn")
    detection = rcnn_detector.objects_detected

    #grasping = pickerUpper.pickUpObj(0,detection.object_clouds,detection.labels_text,detection.full_pcl)
    
    return


    print(detection.object_clouds)
    print(detection.labels_text)
    
    if len(detection.labels_text) == 0:
        return

    pickerUpper = moveObj()
    pickerUpper.pickUpObj(0,detection.object_clouds,detection.labels_text,approach="front")

    # Calculate Pose
    obj_poses = []
    pose_array_msg = PoseArray()
    pose_array_msg.header.stamp = rospy.Time.now()
    pose_array_msg.header.frame_id = "xtion_rgb_optical_frame"

    for obj_cloud, obj_index in zip(detection.object_clouds,detection.labels_text):
        o3d_cloud = convertCloudFromRosToOpen3d(obj_cloud)

        if o3d_cloud is None:
            continue

        mean, _ = o3d_cloud.compute_mean_and_covariance()
        #print(mean, obj_index.data)
        obj_poses.append(mean)
        

    # Publish Poses

        # Create a PoseStamped message
        pose_msg = Pose()

        # Set the position (example: x, y, z)
        pose_msg.position.x = mean[0]
        pose_msg.position.y = mean[1]
        pose_msg.position.z = mean[2]

        # Set the orientation (example: quaternion)
        pose_msg.orientation.x = 0
        pose_msg.orientation.y = 0
        pose_msg.orientation.z = 0
        pose_msg.orientation.w = 1

        pose_array_msg.poses.append(pose_msg)

    # Publish the pose
    pub_pose_array.publish(pose_array_msg)
    print(pose_array_msg)

    # Generate Grasp Poses
    sg = getGrasps()
    all_possible_grasps = []
    for obj_pose in pose_array_msg.poses:
        pose = PoseStamped()
        pose.header.frame_id = "xtion_rgb_optical_frame"
        pose.pose = obj_pose

        pose = transform_pose(pose,"base_footprint")
        print(pose)
        #pub_pose.publish(pose)

        possible_grasps = sg.getGrasps(pose,approach= "front",publish=True)
        
        all_possible_grasps.append(possible_grasps) # Publish Grasp Poses (done automatically) # Order Grasps from Above to Below

    
    # Input All Obstacles Detected

    # Calculate Path
        print(possible_grasps[0])
        controller = robotControlPlanner()
        #possible_grasps= rotate_grasp_pose(possible_grasps)
        print(possible_grasps[0])
        plan = controller.grasp_object(obj_pose, possible_grasps)
        print(plan)
        if plan is True:
            return

    # If not successful try Next Pose (x10)

    # Publish Path

    # Perform Path

    # Move Gripper Down

    # Close Gripper

    # Move Gripper Up


if __name__ == '__main__':
    main()