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
from control_msgs.msg import PointHeadActionGoal, PointHeadAction
from geometry_msgs.msg import PoseStamped
"""
Move the head to a specific pan-tilt angle configuration.
Arguments:
    - pan angle
    - tile angle
"""
class lookAtClosestPerson(AbstractAction):

    #TODO

    def _start_action(self):
        rospy.loginfo('Head movement action to closest person ' + " ".join(self.params) + ' ...')
        #self.starting_time = rospy.Time.now()

        self.ac = actionlib.SimpleActionClient("/head_controller/point_head_action", PointHeadAction)
        rospy.loginfo("Connecting to /head_controller/point_head_action AS...")
        self.ac.wait_for_server()
        rospy.loginfo("Connected.")

        self.last_sent = rospy.get_time()

        self.sub = rospy.Subscriber("/people_tracker/pose", PoseStamped, self.__on_pose_cb)

    def __on_pose_cb(self, msg):
        if (rospy.get_time() - self.last_sent) < 2.:
            return

        # create goal
        self.goal = PointHeadActionGoal()
        # rospy.loginfo(self.goal)
        self.goal.goal.pointing_frame = "xtion_optical_frame"
        self.goal.goal.pointing_axis.x = 0.0
        self.goal.goal.pointing_axis.y = 0.0
        self.goal.goal.pointing_axis.z = 1.0
        self.goal.goal.target.header.frame_id = msg.header.frame_id
        self.goal.goal.target.point.x = msg.pose.position.x
        self.goal.goal.target.point.y = msg.pose.position.x
        self.goal.goal.target.point.z = 1.7

        self.last_sent = rospy.get_time()
        
        # send goal
        self.ac.send_goal(self.goal.goal, done_cb=self._on_look_done)
        rospy.loginfo("Waiting for result...")
        

    def _on_look_done(self, goalState, result):
        print("Head movement DONE", goalState, result)


    def _stop_action(self):

        self.sub.unregister()
        self.ac.cancel_all_goals()
        self.params.append("done")
        rospy.loginfo('STOPPED move head action')

    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True

        return reached