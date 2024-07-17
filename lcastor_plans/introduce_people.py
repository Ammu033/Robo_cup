import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + "/scripts")
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import time
import pnp_cmd_ros
from pnp_cmd_ros import *
from look_for_person import look_for_person
import rospy


def introduce_people(p, person1, person2):
    rospy.log(f'introducing_{person1}_to_{person2}')
    x_y_of_people = [
        str(rospy.get_param(person1 + "/x")),
        str(rospy.get_param(person1 + "/y")),
    ]
    person1_found = look_for_person(p, person1)
    person2_found = look_for_person(p, person2)
    person1_w = rospy.get_param(person1 + "/w")
    person2_w = rospy.get_param(person2 + "/w")
    person1_name = rospy.get_param(person1 + "/name")
    person1_drink = rospy.get_param(person1 + "/drink")
    person2_name = rospy.get_param(person2 + "/name")
    person2_drink = rospy.get_param(person2 + "/drink")
    person1_head_angle = rospy.get_param(person1 + "/head_angle")
    person2_head_angle = rospy.get_param(person2 + "/head_angle")

    # intrducve 1 to 2
    # FIXME not sure where it is going at the moment
    # p.exec_action('goto' , "_".join(x_y_of_people) + '_' + str(person1_w) )
    p.exec_action("moveHead", str(person1_head_angle) + "_0.0")
    p.exec_action("speak", "Hi_" + person1_name + ".Please_meet_" + person2_name)
    # FIXME not sure where it is going at the moment
    # p.exec_action('goto' , "_".join(x_y_of_people) + '_' + str(person2_w) )
    p.exec_action("moveHead", str(person2_head_angle) + "_0.0")
    # p.exec_action('goto' , "_".join(x_y_of_people) + '_' + str(person1_w) )
    p.exec_action("moveHead", str(person1_head_angle) + "_0.0")
    p.exec_action("speak", "Their_favourite_drink_is" + person2_drink)

    # intrducve 2 to 1
    # p.exec_action('goto' , "_".join(x_y_of_people) + '_' + str(person2_w) )
    # p.exec_action('moveHead', str(person2_head_angle) + '_0.0')
    # p.exec_action('speak' , 'Hi_' + person2_name + '.Please_meet_' + person1_name)
    # p.exec_action('goto' , "_".join(x_y_of_people) + '_' + str(person1_w) )
    # p.exec_action('moveHead', str(person1_head_angle) + '_0.0')
    # p.exec_action('goto' , "_".join(x_y_of_people) + '_' + str(person2_w) )
    # p.exec_action('moveHead', str(person2_head_angle) + '_0.0')
    # p.exec_action('speak' , 'Their_favourite_drink_is' + person1_drink)


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    introduce_people(p, "guest1", "guest2")

    p.end()
