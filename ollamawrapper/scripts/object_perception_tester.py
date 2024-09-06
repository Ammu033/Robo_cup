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

sys.path.insert(1, os.path.join(os.path.dirname(__file__), "..", "src", "capabilities"))
print(os.listdir( os.path.join(os.path.dirname(__file__), "..", "src", "capabilities")))
import rag_gpsr_helpers

p = PNPCmd()
p.begin()

rag_gpsr_helpers.identify_objects_primitive(p, input("Input what to look for: "))
rospy.loginfo("Rosparam value: " + rospy.get_param("/gpsr/saved_information"))

p.end()

