import rospy
from pick_up_object.srv import DetectObjects
from lcastor_grasping.utils.classifyObjs import classifyObjs
from geometric_grasp.utils import convertCloudFromRosToOpen3d
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros
import copy

class detectObjs:
    def __init__(self,method = "rcnn"):
        
        self.objects_detected = DetectObjects
        self.categories = []
        # Initialize the transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        if method == "rcnn":
            self.detect = rospy.ServiceProxy('detect_objects', DetectObjects)
            self.refresh = self.detect_objs_mask_rcnn
            self.classify = classifyObjs

        elif method == "cnos":
            self.detect = rospy.ServiceProxy('sam6d/detect_objects', DetectObjects)
            self.refresh = self.detect_objs_cnos
        else:
            print("Method Not Recognised")
        self.refresh(orderObj=True)

    

    def detect_objs_mask_rcnn(self,orderObj=True):
        """
            Calls detection server (Mask-RCNN)
        """
        rospy.loginfo('waiting for detect_objects')
        rospy.wait_for_service('detect_objects', timeout=10)
        
        try:
            self.objects_detected = self.detect()
            rospy.loginfo('detection done!')
            if orderObj:
                self.objects_detected = self.orderObjByDistance(self.objects_detected)
            self.categories = self.classify.classify(self.objects_detected.objects_detected.labels_text)
            
            return self.objects_detected, self.categories
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)
            return DetectObjects,[]


    def detect_objs_cnos(self,orderObj=True):
        """
            Calls detection server (CNOS (sam 6D))
        """
        rospy.loginfo('waiting for detect_objects')
        rospy.wait_for_service('sam6d/detect_objects', timeout=10)
        
        try:
            self.objects_detected = self.detect()
            
            rospy.loginfo('detection done!')
            rospy.loginfo(self.objects_detected.categories)
            if orderObj:
                self.objects_detected = self.orderObjByDistance(self.objects_detected)
            
            self.objects_detected = self.transformPointCloud2s(self.objects_detected)
            rospy.loginfo(self.objects_detected.labels_text)
            #rospy.loginfo(self.objects_detected.full_pcl)
            self.categories = self.objects_detected.categories
            return self.objects_detected, self.categories
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)
            return DetectObjects,[]

    
    def orderObjByDistance(self,detection):
        rospy.loginfo("")
        length = len(detection.object_clouds)
        new_order = []
        reordered_detections = detection
        distances = []
        mask = []

        # for all objects
        i = 0
        #print(detection.object_clouds)
        for obj in detection.object_clouds:

        # get poses
            _, pose = self.getBoundingBox(obj)
            #print("Object Pose: ", pose)

            # handle None Clouds
            if pose == None:
                mask.append(0)
                new_order.append(i)
                continue
            
            mask.append(1)

        # calculate distance
            distance = np.sqrt(np.power(pose[0],2)+np.power(pose[1],2)+np.power(pose[2],3))
        
        # add to array
            distances.append(distance)
            new_order.append(i)
            i += 1

        #print(mask)
        #print(new_order)
        # generate indexing list
        new_order = [x for _, x in sorted(zip(distances, new_order))]
        print("Object Distances:",distances)
        print("Object Order:",new_order)
        print("Object Mask:",mask)
    
        # reorder categories (if correct length)
        if length is len(detection.categories):
            reordered_detections.categories = self.reorderDetections(detection.categories,mask,new_order)

        # reorder objects
        if length is len(detection.object_clouds):
            reordered_detections.object_clouds = self.reorderDetections(detection.object_clouds,mask,new_order)

        # reorder scores
        if length is len(detection.scores):
            reordered_detections.scores = self.reorderDetections(detection.scores,mask,new_order)

        # reorder labels_text
        if length is len(detection.labels_text):
            reordered_detections.labels_text = self.reorderDetections(detection.labels_text,mask,new_order)
        
        return reordered_detections
    
    def reorderDetections(self,reorder_list,mask,new_order):
        # Filter reorder_list and new_order based on the mask
        filtered_reorder_list = [item for item, m in zip(reorder_list, mask) if m == 1]
        filtered_new_order = [item for item, m in zip(new_order, mask) if m == 1]
        
        #print(filtered_reorder_list)
        #print(filtered_new_order)
        
        # Reorder the filtered_reorder_list according to filtered_new_order
        reordered_list = [filtered_reorder_list[i] for i in filtered_new_order]
        
        return reordered_list

    def getObjectPose(self,point_cloud_msg, target_frame="base_footprint"):
        o3d_cloud = convertCloudFromRosToOpen3d(point_cloud_msg)
        
        if o3d_cloud is None:
            rospy.logwarn("NONE POINT CLOUD DETECTED")
            print(o3d_cloud)
            return None, None
        
        bounding_box, object_pose = self.getBoundingBox(point_cloud_msg)
        mean, _ = o3d_cloud.compute_mean_and_covariance()

        print(object_pose)
        
        pose_stamped_msg = self.getPoseStamped(object_pose,target_frame)

        return pose_stamped_msg, bounding_box
    
    def getBoundingBox(self,point_cloud_msg):
        #print(type(o3d_cloud))
        o3d_cloud = convertCloudFromRosToOpen3d(point_cloud_msg)
        if o3d_cloud is None:
            rospy.logwarn("NONE POINT CLOUD DETECTED")
            print(o3d_cloud)
            return None, None
        
        # Convert Open3D point cloud to numpy array
        points = np.asarray(o3d_cloud.points)

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
    
    def transformPointCloud2s(self,detected_objects,target_frame="base_footprint"):
        
        out_detect = copy.deepcopy(detected_objects)
        
        out_detect.full_pcl = self.transformPointCloud2(detected_objects.full_pcl,target_frame)

        out_detect.object_clouds = []
        out_detect.scores = []
        out_detect.labels_text = []
        
        for obj,score,label in zip(detected_objects.object_clouds,detected_objects.scores,detected_objects.labels_text):
            o3d_cloud = convertCloudFromRosToOpen3d(obj)

            if o3d_cloud is None:
                rospy.logwarn("NONE POINT CLOUD DETECTED REMOVING")
                continue

            out_detect.object_clouds.append(self.transformPointCloud2(obj,target_frame))
            out_detect.scores.append(score)
            out_detect.labels_text.append(label)

        return out_detect

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
        