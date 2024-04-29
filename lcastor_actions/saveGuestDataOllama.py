import rospy
from tf import TransformListener
from AbstractAction import AbstractAction
from std_msgs.msg import String
import json


class saveGuestDataOllama(AbstractAction):

    def _start_action(self):
        self.tf = TransformListener()
        rospy.loginfo("Starting Reception Action ")
        # person = rospy.get_param('/person_to_deal')
        check_param = self.param[0].lower()
        set_param = self.param[1].lower()

        if check_param == "setname":
            rospy.set_param(set_param + "/head_angle", 0.0)
            rospy.set_param(
                set_param + "/name",
                rospy.wait_for_message("ollama_name", String, timeout=60).data,
            )
        elif check_param == "setdrink":
            rospy.set_param(
                set_param + "/drink",
                rospy.wait_for_message("ollama_drink", String).data,
            )

        self.params.append("done")

    def get_position(self):
        now = rospy.Time()
        self.tf.waitForTransform("/base_link", "/map", now, rospy.Duration(4.0))
        return self.tf.lookupTransform("/base_link", "/map", now)

    def _stop_action(self):
        self.params.append("done")

    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
