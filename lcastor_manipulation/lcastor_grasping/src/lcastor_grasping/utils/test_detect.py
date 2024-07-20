
import rospy
from lcastor_grasping.utils.detectObjs import detectObjs

def do_somthin():
     rospy.init_node('testing_grasping', anonymous=True)
     object_detector = detectObjs(method="cnos")
     print(len(object_detector.objects_detected.object_clouds))
     print(object_detector.objects_detected.categories)
     exit()
 

do_somthin()