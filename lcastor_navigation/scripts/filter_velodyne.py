#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber
import pcl
import pcl_ros
from pcl import PointXYZ

def pointcloud_callback(pc_msg):
    cloud = pcl.PointCloud()
    pcl_ros.pointCloud2ToPointCloud(pc_msg, cloud)

    # Filter based on distance
    filtered_cloud = pcl.PointCloud()
    distance_threshold_min = 2.0  # Minimum distance threshold
    distance_threshold_max = 5.0  # Maximum distance threshold
    filter_indices = []
    for i in range(cloud.size):
        point = cloud[i]
        distance = point.getArray3fMap().norm()
        if distance >= distance_threshold_min and distance <= distance_threshold_max:
            filter_indices.append(i)

    filtered_cloud.from_array(cloud.to_array()[filter_indices])

    # Convert filtered point cloud back to PointCloud2
    filtered_msg = pcl.PointCloud().to_msg()
    pcl_ros.pointCloud2FromPointCloud(filtered_cloud, filtered_msg)

    # Publish the filtered point cloud
    filtered_pub.publish(filtered_msg)

if __name__ == '__main__':
    rospy.init_node('pointcloud_filter_node')

    pc_sub = Subscriber('/velodyne_points', PointCloud2)
    ts = ApproximateTimeSynchronizer([pc_sub], queue_size=10, slop=0.1)
    ts.registerCallback(pointcloud_callback)

    filtered_pub = rospy.Publisher('/velodyne_points_filtered', PointCloud2, queue_size=1)

    rospy.spin()
