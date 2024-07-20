
import rospy
from lcastor_grasping.utils.robotControlPlanner import robotControlPlanner

def do_somthin():
     rospy.init_node('testing_grasping', anonymous=True)
     planner = robotControlPlanner(group_name="arm")
     joint_cmd_list=[[2.678042047569375, 1.0188283670051694, -1.950075317204145, 0.15194301862522813, -1.801854583596111, -1.1080452565764802, 0.6317888675351815],
                     [2.678532927932626, 1.0192578873230145, 0.0934320229912869, 0.15251059904523773, -1.8019916785176704, -1.1080211723335036, -0.018072555435998883],
                     [2.678394867830462, 1.019549347538695, 0.0939535833772416, -0.32267693259353575, -1.8028364796559289, -1.1080044986268274, -0.017729818132100218]]
     joint_cmd_list.reverse()
     planner.setArmState(joint_cmd_list)
     exit()
 

do_somthin()