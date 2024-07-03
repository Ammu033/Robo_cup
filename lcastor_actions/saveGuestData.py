import rospy
from tf import TransformListener
from AbstractAction import AbstractAction
from std_msgs.msg import String


class saveGuestData(AbstractAction):

    def _start_action(self):
        self.tf = TransformListener()

        rospy.loginfo("saving guest data")
        rospy.loginfo("saveGuestData: saving the param...")
        rospy.loginfo(f"param = {self.params}")

        setparam = self.params[0].lower()
        guestparam = self.params[1].lower()

        if setparam == "setname":
            rospy.set_param(guestparam + "/head_angle", 0.0)
            rospy.set_param(guestparam + "/name", self.params[2].lower())
        elif setparam == "setdrink":
            rospy.set_param(guestparam + "/drink", "_".join(self.params[:]).lower())
        elif setparam == "setheadangle":
            rospy.set_param(guestparam + '/head_angle' , self.params[2])
        elif setparam  == "setloc":
            p,o  = self.get_position()
            rospy.set_param(guestparam + '/x', p[0] )
            rospy.set_param(guestparam + "/y", p[1] )
            rospy.set_param(guestparam + '/w', o[3])
            rospy.set_param(guestparam + '/head_angle', 0.0)
        elif setparam == "setid":
            rospy.set_param(guestparam + "/id", rospy.get_param('LastSavedid'))
            rospy.set_param('learn' , 0)
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
