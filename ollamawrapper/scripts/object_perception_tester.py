import sys
import os

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import time
import pnp_cmd_ros
from pnp_cmd_ros import *
import rospy

sys.path.insert(1, os.path.join(os.path.basename(__file__), "..", "src"))
from capabilities import gpsr

p = PNPCmd()
p.begin()

gpsr.identify_objects(p, input("Input what to look for: "))
rospy.loginfo("Rosparam value: " + rospy.get_param("/gpsr/saved_information"))

p.end()

