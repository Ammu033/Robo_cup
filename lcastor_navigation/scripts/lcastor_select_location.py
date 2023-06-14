#!/usr/bin/env python

import rospy
from lcastor_navigation.srv import FindLocation, FindLocationResponse

NODE_NAME = "destination_selector"
NODE_RATE = 10 # Hz

class DestinationSelector():
    """
    This class select in which location the robot needs to be sent by query for the object is looking for.
    """

    def __init__(self):
        """
        Constructor, initialise the service
        """

        self.obj_dict = {"cup": "kitchen",
                         "bed": "bedroom",
                         "bagpack" : "livingroom"}
        self.location_srv = rospy.Service("select_location", FindLocation, self.find_location)

    def find_location(self, req):
        if req.object in self.obj_dict:
            return FindLocationResponse(self.obj_dict[req.object])
        else:
            return FindLocationResponse("None")

        

if __name__ =="__main__":

    # Init node
    rospy.init_node(NODE_NAME)

    # Set node rate
    rate = rospy.Rate(NODE_RATE)

    destination_selector = DestinationSelector()

    while not rospy.is_shutdown():
        rate.sleep()