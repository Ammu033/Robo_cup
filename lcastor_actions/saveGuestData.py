import rospy
from tf import TransformListener
from AbstractAction import AbstractAction
from std_msgs.msg import String
import json

class saveGuestData(AbstractAction):

    def _start_action(self):
        self.tf = TransformListener()
        rospy.loginfo('Starting Reception Action ')
        #person = rospy.get_param('/person_to_deal')
        if(self.params[0].lower() =="setloc"):
            p , o  = self.get_position()
            rospy.set_param(self.params[1].lower()+'/x' , p[0] )
            rospy.set_param(self.params[1].lower() + "/y" , p[1] )
            rospy.set_param(self.params[1].lower() + '/w' ,  o[3])
        if(self.params[0].lower() == "setheadangle"):
            rospy.set_param(self.params[1].lower() + '/head_angle' , self.param[2])
        elif(self.params[0].lower() == "setid"):
            rospy.set_param(self.params[1].lower() + "/id" , rospy.get_param('LastSavedid'))
            rospy.set_param('learn' , 0)
        elif(self.params[0].lower() == "setname"):
            rospy.set_param(self.params[1].lower() + '/head_angle' , 0.0)
            rospy.set_param(self.params[1].lower() + "/name" , rospy.wait_for_message('guest_name' , String).data)
        elif(self.params[0].lower() == "setdrink"):
            rospy.set_param(self.params[1].lower() + "/drink" , rospy.wait_for_message('guest_drink' , String).data)
        #elif(self.params[0] == "savedata"):
        #    json_obj  = json.dumps(rospy.get_param(self.params[1]) , indent=4)
        #    with open(self.params[0] + '.json' , "w") as outfile:
        #    	outfile.write(json_obj)
        self.params.append('done')

    def get_position(self):
        now = rospy.Time()
        self.tf.waitForTransform('/base_link'  , "/map" , now , rospy.Duration(4.0))
        return(self.tf.lookupTransform("/base_link" , "/map" , now))
    
    def _stop_action(self):
        self.params.append('done')
    
    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
