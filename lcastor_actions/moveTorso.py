import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
from AbstractAction import AbstractAction
from pnp_msgs.srv import PNPCondition, PNPConditionValue
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class moveTorso(AbstractAction):

    def _start_action(self):
        rospy.loginfo('Moving torso to ' + " ".join(self.params))
        self.torso_cmd = rospy.Publisher(
			'/torso_controller/command', JointTrajectory, queue_size=1)
		jt = JointTrajectory()
		jt.joint_names = ['head_1_joint', 'head_2_joint']
		jtp = JointTrajectoryPoint()
		jtp.positions = [float(self.params[0]), float(self.params[1])]
		jtp.time_from_start = rospy.Duration(2.0)
		jt.points.append(jtp)
		self.head_cmd.publish(jt)
		rospy.loginfo("Done.")

    def _stop_action(self):
        rospy.loginfo('Finished moving torso')

    @classmethod
    def is_goal_reached(cls, params):
        ''' check conditions CurrentGoal and GoalStartingTime '''
        condition_value_sp = rospy.ServiceProxy("/PNPConditionValue", PNPConditionValue)
        condition_eval_sp = rospy.ServiceProxy("/PNPConditionEval", PNPCondition)

        time = 5 #seconds
        if len(params) > 0:
            try:
                time = int(params[0])
            except ValueError:
                return False

        # The goal is reached when we have finished saying what we actually want to say
        current_goal_cond = "CurrentGoal_" + cls.__name__ + "_" + "_".join(params)
        if condition_eval_sp(current_goal_cond).truth_value:
            # check that the elapsed time is enough
            starting_time = condition_value_sp("GoalStartingTime").value
            if starting_time != "None":
                elapsed_time = rospy.Time.now() - rospy.Time.from_sec(float(starting_time))
                if elapsed_time.to_sec() > time:
                    return True

        return False
