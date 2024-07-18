import os
import sys
from ollamamessages.msg import WhisperListening

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import time
import pnp_cmd_ros
from pnp_cmd_ros import *
from std_msgs.msg import Bool
import rospy

def gpsr():
    listening_pub = rospy.Publisher("/stt/listening", WhisperListening, queue_size=1)
    listening_pub.publish(listening=True)

if __name__ == "__main__":
    gpsr()
