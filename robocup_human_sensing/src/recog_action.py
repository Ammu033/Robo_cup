import rospy
import os
from AbstractAction import AbstractAction 
class Recog(AbstractAction) :
    def _start_action(self):
        rospy.set_param('learn' , 0)
        os.system('gnome-terminal -x rosrun robocup_human_sensing people_identifier.py')
    def _stop_action(self):
        os.system("rosnode kill people_identifier")
