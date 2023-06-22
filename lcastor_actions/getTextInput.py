import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
from AbstractAction import AbstractAction
import actionlib
from std_msgs.msg import String

"""
Move the head to a specific pan-tilt angle configuration.
Arguments:
    - pan angle
    - tile angle
"""
class getTextInput(AbstractAction):

    def _start_action(self):
        rospy.loginfo('Obtaining text input from UI: ' + " ".join(self.params) + ' ...')
        #self.starting_time = rospy.Time.now()

        self.input = rospy.Publisher("/interface/showmodalInput", String, queue_size=10)
        self.input_close = rospy.Publisher("/interface/showmodalInputclose", String, queue_size=10)
        rospy.Subscriber("/alternative_stt", String, self.__alt_sentence_cb)

        self.input.publish(str(" ".join(self.params)))

    def __alt_sentence_cb(self, msg):
        rospy.set_param("textInput", {"text": msg.data, "time_secs": rospy.get_time()})

        self.params.append("done")

    def _stop_action(self):

        self.input_close.publish("")
        rospy.loginfo('STOPPED getting text input')

    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True

        return reached