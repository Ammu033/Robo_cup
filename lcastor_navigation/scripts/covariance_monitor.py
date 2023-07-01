#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32, String
from lcastor_navigation.msg import LocalizationStatus
from actionlib_msgs.msg import GoalID

GOOD_LOCALIZATION_THR    = 1e-4
PERFECT_LOCALIZATION_THR = 1e-6

hysteresis = 5
localization_start_state = None
det_start_value = None
loc_history_state = [localization_start_state] * 2
det_hist_value = [det_start_value] * (hysteresis * 2)

#det_pub = None
#feed_pub = None

def pose_cb(msg):
    loc_state = LocalizationStatus()
    loc_state.status = LocalizationStatus.MOST_PROBABLY_LOCALIZED
    loc_state.stamp = rospy.Time.now()

    cov = msg.pose.covariance
    cov = np.reshape(np.array(cov), (6,6))
    # remove all zero columns and rows
    cov = cov[:,~np.all(cov == 0, axis=0)]
    cov = cov[~np.all(cov == 0, axis=1)]
    rospy.loginfo(cov)
    det = np.absolute(np.linalg.det(cov))
#    det_pub.publish(det)
    loc_state.det_cov = det

    # update determinant history values
    det_hist_value.pop(0)
    det_hist_value.append(det)

#    rospy.loginfo(det_hist_value)
    if all([el is not None for el in det_hist_value]):
        # update localization state
        loc_history_state = [np.average(det_hist_value[:hysteresis]), np.average(det_hist_value[hysteresis:])]
        for i in range(len(loc_history_state)):
            if loc_history_state[i] < PERFECT_LOCALIZATION_THR:
                loc_history_state[i] = LocalizationStatus.PERFECTLY_LOCALIZED
            elif loc_history_state[i] < GOOD_LOCALIZATION_THR:
                loc_history_state[i] = LocalizationStatus.MOST_PROBABLY_LOCALIZED
            else:
                loc_history_state[i] = LocalizationStatus.MISLOCALIZED

        status_text = ""
        # Take a decision according to the states
        # - loc improving
        if loc_history_state[1] > loc_history_state[0]:
            rospy.loginfo("Localization is improving")
#            feed_pub.publish("Localization is improving")
            status_text = "Localization is improving"
            loc_state.status = LocalizationStatus.MOST_PROBABLY_LOCALIZED
        # - loc is getting worse
        elif loc_history_state[1] < loc_history_state[0]:
            rospy.loginfo("Localization is getting worse")
#            feed_pub.publish("Localization is getting worse")
            status_text = "Localization is getting worse"
            loc_state.status = LocalizationStatus.MOST_PROBABLY_LOCALIZED
            # TODO do something?
        # - loc is stable
        else:
            if loc_history_state[0] == LocalizationStatus.MISLOCALIZED:
                rospy.loginfo("Localization is stably bad")
#                feed_pub.publish("Localization is stably bad")
                status_text = "Localization is stably bad"
                if det_hist_value[-1] > PERFECT_LOCALIZATION_THR:
                    loc_state.status = LocalizationStatus.MISLOCALIZED
                else:
                    loc_state.status = LocalizationStatus.MOST_PROBABLY_LOCALIZED

            elif loc_history_state[0] == LocalizationStatus.MOST_PROBABLY_LOCALIZED:
                rospy.loginfo("Localization is probably good")
#                feed_pub.publish("Localization is stably good")
                status_text = "Localization is probably good"
                loc_state.status = LocalizationStatus.MOST_PROBABLY_LOCALIZED

                # TODO do something?
            else:
                rospy.loginfo("Localization is stably perfect")
#                feed_pub.publish("Localization is stably perfect")
                status_text = "Localization is stably perfect"
                loc_state.status = LocalizationStatus.PERFECTLY_LOCALIZED

        loc_state.text = status_text

    loc_pub.publish(loc_state)
    if loc_state.status == LocalizationStatus.MISLOCALIZED:
        cancel_navigation_pub.publish(GoalID())
        cancel_movebase_pub.publish(GoalID())

if __name__ == "__main__":
    rospy.init_node("amcl_covariance_monitor")

    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback=pose_cb)

    loc_pub = rospy.Publisher("/localization_status", LocalizationStatus, queue_size=1, latch=True)

    cancel_navigation_pub = rospy.Publisher("/topological_navigation/cancel", GoalID, queue_size=1, latch=True)
    cancel_movebase_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1, latch=True)
#    det_pub = rospy.Publisher("/amcl_cov_det", Float32, queue_size=1, latch=True)
#    feed_pub = rospy.Publisher("/amcl_localization_feedback", String, queue_size=1, latch=True)

    rospy.spin()

    # rospy.Rate(1) #Hz
    #
    # while not rospy.is_shutdown():
    #     det_hist_value.insert(-1, det_curr_value)
    #     det_hist_value.pop()
