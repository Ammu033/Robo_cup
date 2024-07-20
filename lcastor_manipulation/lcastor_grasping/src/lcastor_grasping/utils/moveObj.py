import tf2_ros
import tf2_geometry_msgs
from lcastor_grasping.utils.sphericalGraspsServer import SphericalGrasps
from lcastor_grasping.utils.robotControlPlanner import robotControlPlanner
from geometric_grasp.utils import convertCloudFromRosToOpen3d
from geometry_msgs.msg import Pose, PoseStamped, Vector3, PoseArray
import rospy
import tf.transformations as tf
import numpy as np
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from contact_graspnet.srv import GenerateGrasps, GenerateGraspsResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import Grasp, GripperTranslation

class getGrasps():
    def __init__(self, method="Spherical"):
        if method == "Spherical":
            self.sg = SphericalGrasps()
            self.getGrasps = self.getSphericalGrasps

        elif method == "ContactGraspnet":
            self.getGrasps = self.getContactGraspnetGrasps
            self.graspPub = rospy.Publisher(
            '/grasp_poses', Grasp, latch=True,queue_size=1)

        else:
            print("Method Not Recognised")

          
    def getSphericalGrasps(self, object_pose, approach = "top", publish=True):
        
        if approach == "front":            
            object_pose.pose.position.x = object_pose.pose.position.x - 0.01 # depth offset for gripping above
            object_pose.pose.position.y = object_pose.pose.position.y - 0.03 # left/right offset
            object_pose.pose.position.z = object_pose.pose.position.z #+ 0.1 # offset for gripping above

        elif approach == "top":
            object_pose.pose.position.x = object_pose.pose.position.x #+ 0.065 # depth offset for gripping above
            object_pose.pose.position.y = object_pose.pose.position.y #- 0.03 # left/right offset
            object_pose.pose.position.z = object_pose.pose.position.z + 0.15 # offset for gripping above


        # if publish is True:
        possible_grasps = self.sg.create_grasps_from_object_pose(object_pose)
        # else:
        #     possible_grasps = self.sg.generate_grasp_poses(object_pose)

        if approach == "front":   
            print(type(possible_grasps[1]))
            possible_grasps = self.rotate_grasp_pose(possible_grasps)
            possible_grasps = possible_grasps[:50]
        elif approach == "top": 
            possible_grasps.reverse()
            possible_grasps = possible_grasps[:50]

        #print(possible_grasps)
        return possible_grasps
    
    def getPutDownGrasps(self,object_pose,approach="front",publish=False):

        possible_grasps = []
        grasp_list = self.getSphericalGrasps(object_pose,approach)
        for grasp in grasp_list:
            possible_grasps.append(self.modify_grasp(grasp))

        return possible_grasps

    # Function to modify the grasp message
    def modify_grasp(self,grasp):
        # Swap the gripper positions in pre_grasp_posture and grasp_posture
        # pre_grasp_posture: open
        grasp.pre_grasp_posture.points[0].positions = [0.01, 0.01]  # Closed positions
        # grasp_posture: closed first and then open
        grasp.grasp_posture.points[0].positions = [0.01, 0.01]  # Closed positions
        grasp.grasp_posture.points[1].positions = [0.05, 0.05]  # Open positions

        # Add a backup retreat after opening
        post_place_retreat = grasp.post_place_retreat
        post_place_retreat.direction.vector = Vector3(1.0, 0.0, 0.0)  # Back up in the negative x-direction
        post_place_retreat.desired_distance = 0.2
        post_place_retreat.min_distance = 0.1

        # Update the grasp message
        grasp.post_place_retreat = post_place_retreat
        
        return grasp
    
    def rotate_grasp_pose(self, grasps, angle=90):
        rotated_grasps = []
        for grasp in grasps:
            # Define the rotation quaternion for a 90-degree rotation around the x-axis
            angle_rad = angle * (3.141592653589793 / 180.0)
            rotation_quaternion = tf.quaternion_from_euler(angle_rad, 0, 0)

            # Current orientation quaternion
            current_orientation = [
                grasp.grasp_pose.pose.orientation.x,
                grasp.grasp_pose.pose.orientation.y,
                grasp.grasp_pose.pose.orientation.z,
                grasp.grasp_pose.pose.orientation.w
            ]

            # Perform the quaternion multiplication
            new_orientation = tf.quaternion_multiply(rotation_quaternion, current_orientation)

            # Update the grasp_pose with the new orientation
            grasp.grasp_pose.pose.orientation.x = new_orientation[0]
            grasp.grasp_pose.pose.orientation.y = new_orientation[1]
            grasp.grasp_pose.pose.orientation.z = new_orientation[2]
            grasp.grasp_pose.pose.orientation.w = new_orientation[3]

            rotated_grasps.append(grasp)
        return rotated_grasps
    
    def changeMethod(self,method):
        if method == "Spherical":
            self.sg = SphericalGrasps()
            self.getGrasps = self.getSphericalGrasps
            return True
        elif method == "ContactGraspnet":
            self.getGrasps = self.getContactGraspnetGrasps
            return True
        else:
            return False
        
    def getContactGraspnetGrasps(self,full_pcl,objs_pcl,index=0,publish=True):
        """
        Arguments:
            full_pcl {PointCloud2} -- full point cloud of the scene
            objs_pcl {PointCloud2[]} -- list with each segmented object pointcloud
        
        Return:
            grasps {GenerateGrasps} -- array of grasps
        """

        print('waiting for generate_grasps_server')
        rospy.wait_for_service('generate_grasps_server', timeout=20)
        try:
            grasp_poses = rospy.ServiceProxy('generate_grasps_server', GenerateGrasps)
            resp = grasp_poses(full_pcl, objs_pcl)
            print('generates grasps done!')
            grasp_contact_points = resp.grasp_contact_points
            grasp_poses = resp.all_grasp_poses[index]
            scores = resp.all_scores
            
            #print(type(grasp_poses))

            possible_grasps = self.makeGraspMsg(grasp_poses)

            possible_grasps = self.rotate_grasp_pose(possible_grasps)

            #print ("possible grasps:",possible_grasps)
            #publish = False
            
            
            return possible_grasps
        

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

            return None, None
            

    def makeGraspMsg(self, grasp_poses):
        grasps = []
        

        id = 0
        for pose in grasp_poses.poses:
            grasp = Grasp()
            #print("printing_pose: ",pose)
            grasp.id = "Contact_Grasp_"+str(id)
            id +=1
            # Grasp Pose
            grasp_pose = PoseStamped()
            grasp_pose.header = grasp_poses.header
            grasp_pose.pose = pose
            grasp_pose.pose.position.x = grasp_pose.pose.position.x - 0.01
            
            grasp.grasp_pose = grasp_pose
            
            # Pre-grasp posture
            grasp.pre_grasp_posture = self.createGripperPosture([0.05, 0.05], 2)  # Open position
            
            # Grasp posture
            grasp.grasp_posture = self.createGripperPosture([0.01, 0.01], 4)  # Closed position
            
            # Pre-grasp approach
            frame_id = grasp_poses.header.frame_id
            grasp.pre_grasp_approach = self.createGripperTranslation(frame_id, [1.0, 0.0, 0.0], 0.15, 0.1)
            
            # Post-grasp retreat
            grasp.post_grasp_retreat = self.createGripperTranslation(frame_id, [-1.0, 0.0, 0.0], 0.15, 0.1)
            
            # Post-place retreat (optional)
            grasp.post_place_retreat = self.createGripperTranslation(frame_id, [0.0, 0.0, 1.0], 0.15, 0.1)
            
            grasps.append(grasp)
        
        return grasps

    def createGripperPosture(self, positions, duration):
        posture = JointTrajectory()
        posture.header.stamp = rospy.Time.now()
        posture.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration)
        
        posture.points.append(point)
        return posture

    def createGripperTranslation(self, frame_id, direction, desired_distance, min_distance):
        translation = GripperTranslation()
        translation.direction.header.frame_id = frame_id
        translation.direction.vector.x = direction[0]
        translation.direction.vector.y = direction[1]
        translation.direction.vector.z = direction[2]
        translation.desired_distance = desired_distance
        translation.min_distance = min_distance
        return translation

class moveObj:
    def __init__(self,method = "Spherical", table=0.78, planner_id = "RRTConnectkConfigDefault",debug_without_control=False) -> None:
        # Initialize the transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.grasp = getGrasps(method)
        self.method = method
        
        self.controller = robotControlPlanner(planner_id=planner_id,debug_without_control=debug_without_control)
        self.pub_pose = rospy.Publisher('/object_pose', PoseStamped, queue_size=10)
        self.approach = "front"
        self.table = table
        

    def generateGrasps(self,method):
        pass

    def pickUpObj(self,index, obj_clouds, obj_labels, full_pcl = [] ,approach = "front", obj_frame = "xtion_rgb_optical_frame",target_frame="base_footprint"):
        """
        Function to pick up an object identified by its index in the obj_clouds and obj_labels.

        Parameters:
        -----------
        index : int
            The index of the object to pick up in the obj_clouds and obj_labels lists.
        obj_clouds : list
            A list of point clouds corresponding to detected objects.
        obj_labels : list
            A list of labels corresponding to detected objects.
        full_pcl : list, optional
            A full point cloud (default is an empty list).
        approach : str, optional
            The direction from which to approach the object. Default is "front".
        obj_frame : str, optional
            The frame in which the object cloud is given. Default is "xtion_rgb_optical_frame".
        target_frame : str, optional
            The target frame to transform the object cloud into. Default is "base_footprint".

        Returns:
        --------
        bool
            Returns True if the object was successfully picked up, otherwise False.
        
        Description:
        ------------
        This function takes an index to identify the target object from the lists of object clouds and labels.
        It then processes the point cloud of the object, transforms it if necessary, and attempts to pick up the object
        from the specified approach direction. The function ensures the robot's end effector maintains the correct orientation
        throughout the pick-up process. The success of the operation is indicated by the returned boolean value. 
        Grasp method should be determined when the object is initiliased

        Generated by ChatGPT
        """
        
        if self.isPointcloudEmpty(obj_clouds[index]):
            return False
        
        ### Calculate the object poses and sizes for adding to planning scene
        obstacle_poses = []
        obstacle_sizes = []
        for obj in obj_clouds:
            obj_cloud = self.transformPointCloud2(obj,target_frame)
            pose_stamped_msg,box_size = self.getObjectPose(obj_cloud,target_frame)
            obstacle_poses.append(pose_stamped_msg)
            obstacle_sizes.append(box_size)

        ### Add All Objects and Move to Point in Front

        self.controller.addPlannerObjects(obstacle_poses,obstacle_sizes,obj_labels,ignore=[],table=self.table)

        # Adjust Pose to be in front
        front_pose_msg = obstacle_poses[index]
        front_pose = [front_pose_msg.pose.position.x - 0.2,front_pose_msg.pose.position.y,front_pose_msg.pose.position.z]

        #self.moveTo(pose=front_pose,restrict_orientation=False)

        ### Add all objects back

        self.controller.addPlannerObjects(obstacle_poses,obstacle_sizes,obj_labels,ignore=[index],table=self.table)

        
        print(obstacle_poses[index])
        self.pub_pose.publish(obstacle_poses[index])
        #transformed_pose = self.transformPose(pose_stamped_msg,target_frame)

        ### Generate Poses
        grasp_poses = Grasp

        if self.method == "Spherical":
            grasp_poses = self.grasp.getGrasps(obstacle_poses[index],approach=approach,publish=True)
        elif self.method == "ContactGraspnet":
            if full_pcl == []:
                rospy.logwarn("Provide Full Point Cloud for Graspnet")
                return False
            
            grasp_poses = self.grasp.getGrasps(full_pcl,obj_clouds,index=index,publish=True)

        if len(grasp_poses) > 10:
            print("Move Object Grasps",grasp_poses[:10])
        else:
            print("Move Object Grasps",grasp_poses[:end])

        ### Move to Grasp
        self.graspObject(obstacle_poses[index],grasp_poses)
        self.approach = approach
        
        ### return if caught anything
        return self.objectInGripper()

    def graspObject(self,object_pose,grasp_poses):
        print("BOOP")
        plan = self.controller.grasp_object(object_pose, grasp_poses)


    def objectInGripper(self):
        joint_values = self.controller.gripperState()

        # Define the thresholds for closed and open states
        closed_threshold = 0.011
        open_threshold = 0.043
        print(joint_values)
        # Check if either joint value is more than the closed threshold
        if any(value > closed_threshold for value in joint_values):
            # Check if either joint value is less than the open threshold
            if any(value < open_threshold for value in joint_values):
                return True

        return False
    
    def getObjectPose(self,point_cloud_msg, target_frame="base_footprint"):
        o3d_cloud = convertCloudFromRosToOpen3d(point_cloud_msg)
        
        if o3d_cloud is None:
            return None, None
        
        bounding_box, object_pose = self.getBoundingBox(point_cloud_msg)
        mean, _ = o3d_cloud.compute_mean_and_covariance()

        print(object_pose)
        
        pose_stamped_msg = self.getPoseStamped(object_pose,target_frame)

        return pose_stamped_msg, bounding_box
    
    def getPoseStamped(self,object_pose,target_frame):
        # Create a PoseStamped message
        pose_msg = Pose()

        # Set the position (example: x, y, z)
        pose_msg.position.x = object_pose[0]
        pose_msg.position.y = object_pose[1]
        pose_msg.position.z = object_pose[2]


        # Set the orientation (example: quaternion)
        pose_msg.orientation.x = 0
        pose_msg.orientation.y = 0
        pose_msg.orientation.z = 0
        pose_msg.orientation.w = 1
        #print(mean, obj_index.data)

        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = rospy.Time.now()
        pose_stamped_msg.header.frame_id = target_frame
        pose_stamped_msg.pose = pose_msg

        return pose_stamped_msg

    def getBoundingBox(self,point_cloud_msg):
        #print(type(o3d_cloud))
        o3d_cloud = convertCloudFromRosToOpen3d(point_cloud_msg)
        if o3d_cloud is None:
            return None, None
        
        # Convert Open3D point cloud to numpy array
        points = np.asarray(o3d_cloud.points)
        #remove any points which are the table/below
        #points = points[points[:, 2] > self.table+0.01] 

        # Extract max and min values for x, y, z
        max_x = np.max(points[:, 0])
        min_x = np.min(points[:, 0])
        max_y = np.max(points[:, 1])
        min_y = np.min(points[:, 1])
        max_z = np.max(points[:, 2])
        min_z = np.min(points[:, 2])

        # Calculate the Corners
        corners = np.array([
            [min_x, min_y, min_z],
            [min_x, min_y, max_z],
            [min_x, max_y, min_z],
            [min_x, max_y, max_z],
            [max_x, min_y, min_z],
            [max_x, min_y, max_z],
            [max_x, max_y, min_z],
            [max_x, max_y, max_z]
        ])

        # Form Bounding Box
        size = [max_x-min_x,max_y-min_y,max_z-min_z]
        
        # Calculate the centroid of the bounding box
        centroid_x = (max_x + min_x) / 2.0
        centroid_y = (max_y + min_y) / 2.0
        centroid_z = (max_z + min_z) / 2.0
        center = [centroid_x, centroid_y, centroid_z]

        return size, center

    def transformPose(self,pose_stamped, target_frame):
        """
        Transforms a PoseStamped from its current frame to the target frame.

        :param pose_stamped: PoseStamped to be transformed.
        :param target_frame: Target frame to transform the pose to.
        :return: Transformed PoseStamped.
        """
        

        try:
            # Lookup the transform from the pose's frame to the target frame
            transform = self.tf_buffer.lookup_transform(target_frame, pose_stamped.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

            # Transform the pose
            transformed_pose_stamped = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

            transformed_pose_stamped.pose.orientation.x = 0
            transformed_pose_stamped.pose.orientation.y = 0
            transformed_pose_stamped.pose.orientation.z = 0
            transformed_pose_stamped.pose.orientation.w = 1

            return transformed_pose_stamped
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Transform error: %s", e)
            return None
    
    def isPointcloudEmpty(self, cloud):
        # Check if width and height are zero
        if cloud.width == 0 and cloud.height == 0:
            return True
        return False
    
    def transformPointCloud2(self,point_cloud,target_frame):
        try:
            
            # Lookup the transform from the pose's frame to the target frame
            transform = self.tf_buffer.lookup_transform(target_frame, point_cloud.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

            # Transform the cloud
            transformed_point_cloud = do_transform_cloud(point_cloud, transform)


            return transformed_point_cloud
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Transform error: %s", e)
            return None
        
    def moveTo(self,predefined="safe",pose=[0,0,0],restrict_orientation=True):
        if pose != [0,0,0]:
            pose_str = ', '.join(map(str, pose))
            rospy.loginfo("Moving to Pose: {}".format(pose_str))
            self.controller.moveToPose(pose,restrict_orientation=restrict_orientation)
        else:
            self.controller.moveToPredefined(predefined,restrict_orientation=restrict_orientation)
            
    def putDownObjAtPose(self,pose,target_frame="base_footprint"):
        pose[2] = pose[2] + 0.05
        pose_stamped_msg = self.getPoseStamped(pose,target_frame)
        possible_grasps = self.grasp.getPutDownGrasps(pose_stamped_msg,approach=self.approach)
        self.graspObject(pose_stamped_msg,possible_grasps)

    def putDownObj(self,index=None, obj_clouds=None, obj_labels=None,pose=[0,0,0],approach = "left", obj_frame = "xtion_rgb_optical_frame",target_frame="base_footprint"):
        '''
            puts down an object next to another object
        '''
        
        if self.isPointcloudEmpty(obj_clouds[index]):
            return False

        obstacle_poses = []
        obstacle_sizes = []
        for obj in obj_clouds:
            obj_cloud = self.transformPointCloud2(obj,target_frame)
            if obj_cloud is None:
                continue
            pose_msg, box_size = self.getObjectPose(obj_cloud,target_frame)
            obstacle_poses.append(pose_msg)
            obstacle_sizes.append(box_size)

        self.controller.addPlannerObjects(obstacle_poses,obstacle_sizes,obj_labels,ignore=[index],table=self.table)

        if pose == [0,0,0]:
            # get object pose
            obj_cloud = self.transformPointCloud2(obj_clouds[index],target_frame)
            box_size, pose = self.getBoundingBox(obj_cloud)
            
            if approach == "left":

                # Add the width of the object it is going next to
                pose[1] = pose[1] - box_size[1]
                self.putDownObjAtPose(pose,target_frame)

            elif approach == "front":
                
                # Add the width of the object it is going next to
                pose[0] = pose[0] - box_size[0]
                self.putDownObjAtPose(pose,target_frame)

            # default to right
            else:
                
                # Add the width of the object it is going next to
                pose[1] = pose[1] + box_size[1]
                self.putDownObjAtPose(pose,target_frame)

        self.putDownObjAtPose(pose,target_frame)
        return not self.objectInGripper()
    
    def clear(self):
        self.controller.clear()