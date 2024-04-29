import rospy
from tf import TransformListener
from AbstractAction import AbstractAction
from std_msgs.msg import String
import json


class saveGuestDataOllama(AbstractAction):

    def _start_action(self):
        self.tf = TransformListener()
        # rospy.loginfo("Starting Reception Action ")
        # person = rospy.get_param('/person_to_deal')
        # print('check_param:', check_param)
        # print('set_param:', set_param)
        rospy.loginfo("saveGuestDataOllama: saving the param...")
        rospy.loginfo(f"param = {self.params}")



        if (self.params[0].lower() == "setname"):
        
            rospy.set_param(self.params[1].lower() + "/head_angle", 0.0)
            rospy.set_param(
                self.params[1].lower() + "/name",
                self.params[2].lower(),
            )
        elif self.params[0].lower() == "setdrink":
            rospy.set_param(
                self.params[1].lower() + "/drink",
                self.params[2].lower()
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
