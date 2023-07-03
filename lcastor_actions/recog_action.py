import rospy
import os
from AbstractAction import AbstractAction 
class Recog(AbstractAction) :
    def _start_action(self):
        rospy.set_param('learn' , 0)
        os.system('gnome-terminal -x roslaunch robocup_human_sensing dark.launch')
    def _stop_action(self):
        os.system("rosnode kill people_identifier darknet_ros people_roi")
