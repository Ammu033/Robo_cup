#!/usr/bin/python
import rospy
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
import sensor_msgs.point_cloud2 as pc2

from cv_bridge import CvBridge, CvBridgeError
import cv2

class tiago_camera:
    def __init__(self):
        self.count = 0
        self.point_cloud = None
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("click_pose", PoseStamped, queue_size=10)

        rospy.Subscriber("/xtion/rgb/image_rect_color", Image, self.callback, queue_size=1, buff_size=2058)
        rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.point_callback, queue_size=1)


    def callback(self, data):
        cv2.namedWindow("tiago_image")
        cv2.setMouseCallback("tiago_image", self.publish_point)
        if self.count == 0:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("tiago_image", cv_image)
            cv2.waitKey(1)
        elif self.count > 10:
            self.count = 0
        else:
            self.count += 1

    def point_callback(self, data):
        self.point_cloud = data

    def publish_point(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print('x = %d, y = %d' % (x, y))
            # point_cloud = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
            gen = pc2.read_points(self.point_cloud, uvs=[(x, y)])

            for p in gen:
                print("x : %f y: %f z : %f" % (p[0], p[1], p[2]))

            q = quaternion_from_euler(1.5707, -1.5707, 1.5707)

            if isinstance(p[0], float) and isinstance(p[1], float) and isinstance(p[2], float):
                my_header = Header(stamp=self.point_cloud.header.stamp, frame_id=self.point_cloud.header.frame_id)
                my_pose = Pose(position=Point(x=p[0], y=p[1], z=p[2]), orientation=Quaternion(
                    x=q[0], y=q[1], z=q[2], w=q[3]))
                pose_stamped = PoseStamped(header=my_header, pose=my_pose)

                self.pub.publish(pose_stamped)


def main():
    print("Starting node")
    rospy.init_node('click_pose_publisher')

    TiagoCamera = tiago_camera()

    rospy.spin()


if __name__ == '__main__':
    main()