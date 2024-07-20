import sys
import rospy
from moveit_msgs.msg import PickupAction, PickupGoal
import moveit_commander
from actionlib import SimpleActionClient
from moveit_msgs.msg import PickupAction, PickupGoal
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, OrientationConstraint
import tf.transformations as tf_trans
from geometry_msgs.msg import Pose
from time import sleep
from std_srvs.srv import Empty
import moveit_msgs.msg

class robotControlPlanner():
    def __init__(self,planner_id = "RRTConnectkConfigDefault",debug_without_control=False,group_name="arm_torso"):
        
        self.debug_without_control = debug_without_control
        if debug_without_control:
            return


        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the RobotCommander object which provides information such as the robot's kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()

        # Initialize the PlanningSceneInterface object which provides a remote interface for getting, setting, and updating the robot's internal understanding of the surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()

        # Initialize the MoveGroupCommander object. This object is an interface to one group of joints. In this case, we are using the group of joints in the Tiago's arm.
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.set_planner_id(planner_id)
        
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)
        
        # Configure the planner to only attempt planning once
        self.group.set_num_planning_attempts(1)
        self.group.set_planning_time(5)

        # Allow replanning to increase the odds of a solution
        self.group.allow_replanning(True)

        # Set the goal pose tolerance
        self.group.set_goal_position_tolerance(0.1)
        self.group.set_goal_orientation_tolerance(0.1)

        self.pickup_ac = SimpleActionClient('/pickup', PickupAction)
        self.pickup_ac.wait_for_server()
        self.links_to_allow_contact = rospy.get_param('~links_to_allow_contact', None)
        if self.links_to_allow_contact is None:
            rospy.logwarn("Didn't find any links to allow contacts... at param ~links_to_allow_contact")
        else:
            rospy.loginfo("Found links to allow contacts: " + str(self.links_to_allow_contact))

    def add_objects_to_scene(self,objects):
        return False

    def grasp_object(self,object_pose, grasp_pose):
        if self.debug_without_control:
            return False
        goal = self.createPickupGoal(
			"arm_torso", "part", object_pose, grasp_pose, self.links_to_allow_contact)
        self.pickup_ac.send_goal(goal)
        result = self.pickup_ac.wait_for_result()
        print(result)
        return result
    
    def clear_planning_scene(self):
        if self.debug_without_control:
            return False
        # Get the current known objects in the planning scene
        known_objects = self.scene.get_known_object_names()
        
        # Remove all known objects
        for obj in known_objects:
            self.scene.remove_world_object(obj)
        
        rospy.loginfo("Cleared all objects from the planning scene.")
        
        # Sleep to allow for the objects to be cleared
        sleep(1)


    def createPickupGoal(self, group="arm_torso", target="part",
					 grasp_pose=PoseStamped(),
					 possible_grasps=[],
					 links_to_allow_contact=None):
        if self.debug_without_control:
            return PickupGoal()
        """ Create a PickupGoal with the provided data"""
        pug = PickupGoal()
        pug.target_name = target
        pug.group_name = group
        pug.possible_grasps.extend(possible_grasps)
        pug.allowed_planning_time = 35.0
        pug.planning_options.planning_scene_diff.is_diff = True
        pug.planning_options.planning_scene_diff.robot_state.is_diff = True
        pug.planning_options.plan_only = False
        pug.planning_options.replan = True
        pug.planning_options.replan_attempts = 30
        pug.allowed_touch_objects = []
        pug.attached_object_touch_links = ['<octomap>']
        #pug.attached_object_touch_links.extend(links_to_allow_contact)

        return pug

    def addPlannerObjects(self,object_poses,object_sizes,object_labels,ignore=[],table=0.5):
        if self.debug_without_control:
            return False

        self.clear_octomap()

        ## Add Table
        # Create a PoseStamped for the plane
        plane_pose = PoseStamped()
        plane_pose.header.frame_id = "base_footprint"  # Replace with your desired frame_id
        plane_pose.pose.position.x = 1  # Replace with your desired position
        plane_pose.pose.position.y = 0.0
        plane_pose.pose.position.z = table/2  # Height of the table

        size = [1.0,1.0,table]
        self.scene.add_box("table", plane_pose, size = size)
        
        # Add objects to the scene based on indexes and ignore list
        for i, size in enumerate(object_sizes):
            
            if object_poses[i] is None:
                continue
            
            if i in ignore:
                continue  # Skip objects not in indexes or in ignore list

            print("Object Pose Type: ",type(object_poses[i]))
            print("Object Index Type: ",type(object_labels[i]))
            # Ensure the object_id is an ASCII string
            if isinstance(object_labels[i].data, str):
                object_id = object_labels[i].data.encode('ascii', 'ignore').decode('ascii')
            self.scene.add_box(object_id, object_poses[i],size)
        
        self.print_scene_objects()
    
    def print_scene_objects(self):
        if self.debug_without_control:
            return False
        # Get the list of object names from the planning scene
        object_names = self.scene.get_objects()

        # Print each object name
        rospy.loginfo("Objects in the planning scene:")
        for name in object_names:
            rospy.loginfo(f"- {name}")

    def applyOrientationConstraint(self,current_orientation,eef_link,xyz=[0.1,0.1,3.6]):
        if self.debug_without_control:
            return False
         # Create orientation constraint
        constraint = Constraints()
        constraint.name = "tilt_constraint"
        tilt_constraint = OrientationConstraint()
        tilt_constraint.header.frame_id = "base_footprint"
        tilt_constraint.link_name = eef_link
        tilt_constraint.orientation = current_orientation
        tilt_constraint.absolute_x_axis_tolerance = xyz[0]
        tilt_constraint.absolute_y_axis_tolerance = xyz[1]
        tilt_constraint.absolute_z_axis_tolerance = xyz[2]
        tilt_constraint.weight = 1.0
        constraint.orientation_constraints = [tilt_constraint]
        self.group.set_path_constraints(constraint)

    def moveToPose(self,pose_xyz,restrict_orientation=False,orientation_xyzw=[0,0,0,1]):
        if self.debug_without_control:
            return False
        
        # Get the current pose of the end effector
        eef_link = self.group.get_end_effector_link()
        current_pose = self.group.get_current_pose(eef_link).pose   
        current_orientation = current_pose.orientation

        pose = Pose()
        pose.position.x = pose_xyz[0]  # Example: 40 cm in front of the robot
        pose.position.y = pose_xyz[1]  # Example: centered along the y-axis
        pose.position.z = pose_xyz[2]  # Example: 40 cm above the base
        pose.orientation.x = orientation_xyzw[0]
        pose.orientation.y = orientation_xyzw[1]
        pose.orientation.z = orientation_xyzw[2]
        pose.orientation.w = orientation_xyzw[3]

        if restrict_orientation:
            self.applyOrientationConstraint(current_orientation,eef_link)
            pose.orientation = current_pose.orientation
        

        # Set the target pose
        self.group.set_pose_target(pose)

        # Plan and execute the motion
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        # Clear any path constraints
        self.group.clear_path_constraints()
        self.clear_planning_scene()

        return plan

    def moveToPredefined(self,location = "safe",restrict_orientation=False):
        if self.debug_without_control:
            return False
        if location == "safe":
            rospy.loginfo("Moving to Safe Pose")
            self.moveToPose([0.4,-0.15,1],restrict_orientation)
        elif location == "readyToGrab":
            rospy.loginfo("Moving to Ready to Grab Pose")
            self.moveToPose([0.3,-0.2,1.6],False)
        elif location == "outOfCamera":
            rospy.loginfo("Moving to Out of Camera Pose")
            self.moveToPose([0.4,-0.5,1],restrict_orientation)

    def clear_octomap(self):
        if self.debug_without_control:
            return False
        print('waiting for clear_octomap')
        rospy.wait_for_service('clear_octomap', timeout=10)
        try:
            clear_octo = rospy.ServiceProxy('clear_octomap', Empty)
            resp1 = clear_octo()
            print('clearing octomap done!')
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def clear(self):
        if self.debug_without_control:
            return False

        self.group.clear_path_constraints()
        self.clear_planning_scene()
        self.group.stop()

    def gripperState(self):
        if self.debug_without_control:
            return [0.5, 0.5]
        return self.gripper_group.get_current_joint_values() 
    
    def setArmState(self,joint_cmds_list):
        try:
            for joint_cmd in joint_cmds_list:
                #print(self.group.get_current_joint_values() )
                self.group.set_joint_value_target(joint_cmd)
                plan = self.group.plan()

                # Check if the plan was successful before executing
                if plan and plan[0] != moveit_msgs.msg.MoveItErrorCodes.FAILURE:
                    self.group.go(wait=True)
                    # Ensure that there is no residual movement
                    self.group.stop()
                    # Clear the targets after planning
                    self.group.clear_pose_targets()

                else:
                    rospy.logwarn("Planning failed")
                    return False
        except moveit_commander.MoveItCommanderException as e:
            rospy.logwarn("MoveItCommanderException: {}".format(e))
            return False
        except rospy.ROSInterruptException as e:
            rospy.logwarn("ROSInterruptException: {}".format(e))
            return False
        except Exception as e:
            rospy.logwarn("Unexpected exception: {}".format(e))
            return False
        

        return True