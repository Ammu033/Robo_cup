# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/custom-actions

from typing import Any, Text, Dict, List

from rasa_sdk import Action, Tracker#,  FormValidationAction
from rasa_sdk.executor import CollectingDispatcher
#from rasa_sdk.types import DomainDict
from rasa_sdk.events import AllSlotsReset
from rasa_sdk.events import SlotSet
from rasa_sdk.events import UserUtteranceReverted
#import roslaunch
import rospy
#import time
from std_msgs.msg import Bool, String
        
class ActionConfirmationFalse(Action):
    def name(self) -> Text:
        return "action_confirmation_false"

    def run(self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        #dispatcher.utter_message(text="action_confirmation_false")        
        rospy.init_node('rasa_actions', anonymous=True)
        pub_bool = rospy.Publisher("rasa_confirmation", Bool, queue_size=1)
        rospy.sleep(1)
        pub_bool.publish(False)
        rospy.sleep(1)
        '''	
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        cli_args = ["/home/lcastor/ros_ws/src/LCASTOR/lcastor_nlu/robocup_nlu/launch/bool_publish.launch",'topic_name:=rasa_confirmation','value:=False']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        '''
        return []

class ActionConfirmationConfirmation(Action):
    def name(self) -> Text:
        return "action_confirmation_confirmation"

    def run(self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        #dispatcher.utter_message(text="action_confirmation_false")        
        rospy.init_node('rasa_actions', anonymous=True)	
        pub_bool = rospy.Publisher("rasa_confirmation", Bool, queue_size=1)
        rospy.sleep(1)
        pub_bool.publish(True)
        rospy.sleep(1)
        '''
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        cli_args = ["/home/lcastor/ros_ws/src/LCASTOR/lcastor_nlu/robocup_nlu/launch/bool_publish.launch",'topic_name:=rasa_confirmation','value:=True']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        '''
        return []

class ActionConfirmationThanks(Action):
    def name(self) -> Text:
        return "action_confirmation_thanks"

    def run(self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        #dispatcher.utter_message(text="action_confirmation_false")        
        rospy.init_node('rasa_actions', anonymous=True)	
        pub_bool = rospy.Publisher("rasa_confirmation", Bool, queue_size=1)
        rospy.sleep(1)
        pub_bool.publish(True)
        rospy.sleep(1)
        '''
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        cli_args = ["/home/lcastor/ros_ws/src/LCASTOR/lcastor_nlu/robocup_nlu/launch/bool_publish.launch",'topic_name:=rasa_confirmation','value:=True']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        '''
        return []

class ActionConfirmationArrival(Action):
    def name(self) -> Text:
        return "action_confirmation_arrival"

    def run(self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        #dispatcher.utter_message(text="action_confirmation_false")        
        rospy.init_node('rasa_actions', anonymous=True)	
        pub_bool = rospy.Publisher("rasa_confirmation", Bool, queue_size=1)
        rospy.sleep(1)
        pub_bool.publish(True)
        rospy.sleep(1)
        '''
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        cli_args = ["/home/lcastor/ros_ws/src/LCASTOR/lcastor_nlu/robocup_nlu/launch/bool_publish.launch",'topic_name:=rasa_confirmation','value:=True']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        '''
        return []

class ActionConfirmationAffirmDeny(Action):
    def name(self) -> Text:
        return "action_confirmation_affirm_deny"

    def run(self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        slot_value= tracker.get_slot("person_confirmation")
        
        #dispatcher.utter_message(text="action_confirmation_false")        
        rospy.init_node('rasa_actions', anonymous=True)	
        pub_bool = rospy.Publisher("rasa_confirmation", Bool, queue_size=1)
        rospy.sleep(1)
        pub_bool.publish(True)
        rospy.sleep(1)
        
        pub_bool = rospy.Publisher("person_affirm_deny", Bool, queue_size=1)
        rospy.sleep(1)
        pub_bool.publish(slot_value)
        rospy.sleep(1)
        '''
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        
        cli_args = ["/home/lcastor/ros_ws/src/LCASTOR/lcastor_nlu/robocup_nlu/launch/bool_publish.launch",'topic_name:=rasa_confirmation','value:=True']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        time.sleep(1)
        cli_args = ["/home/lcastor/ros_ws/src/LCASTOR/lcastor_nlu/robocup_nlu/launch/bool_publish.launch",'topic_name:=person_affirm_deny','value:='+str(slot_value)]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        '''
        return []
    
class ActionConfirmationPersonName(Action):
    def name(self) -> Text:
        return "action_confirmation_person_name"

    def run(self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        slot_value= tracker.get_slot("person_name")
        
        #dispatcher.utter_message(text="action_confirmation_false")        
        rospy.init_node('rasa_actions', anonymous=True)	
        pub_bool = rospy.Publisher("rasa_confirmation", Bool, queue_size=1)
        pub_name = rospy.Publisher("guest_name", String, queue_size=1, latch=True)
        rospy.sleep(1)
        pub_bool.publish(True)
        pub_name.publish(slot_value)
        rospy.sleep(1)
        '''
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        
        cli_args = ["/home/lcastor/ros_ws/src/LCASTOR/lcastor_nlu/robocup_nlu/launch/bool_publish.launch",'topic_name:=rasa_confirmation','value:=True']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.shutdown() # to close any previous node
        parent.start()
        time.sleep(1)
        cli_args = ["/home/lcastor/ros_ws/src/LCASTOR/lcastor_nlu/robocup_nlu/launch/string_publish.launch",'topic_name:=guest_name','value:='+slot_value]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.shutdown() # to close any previous node
        parent.start()
        '''
        return []

class ActionConfirmationPersonDrink(Action):
    def name(self) -> Text:
        return "action_confirmation_person_drink"

    def run(self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        slot_value= tracker.get_slot("person_drink")
        
        #dispatcher.utter_message(text="action_confirmation_false")        
        rospy.init_node('rasa_actions', anonymous=True)	
        pub_bool = rospy.Publisher("rasa_confirmation", Bool, queue_size=1)
        pub_drink = rospy.Publisher("guest_drink", String, queue_size=1, latch=True)
        rospy.sleep(1)
        pub_bool.publish(True)

        pub_drink.publish(slot_value)
        rospy.sleep(1)
        '''
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        
        cli_args = ["/home/lcastor/ros_ws/src/LCASTOR/lcastor_nlu/robocup_nlu/launch/bool_publish.launch",'topic_name:=rasa_confirmation','value:=True']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        time.sleep(1)
        cli_args = ["/home/lcastor/ros_ws/src/LCASTOR/lcastor_nlu/robocup_nlu/launch/string_publish.launch",'topic_name:=guest_drink','value:='+slot_value]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        '''       
        return []

class ActionCheckPersonName(Action):
    def name(self) -> Text:
        return "action_check_person_name"

    def run(self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        #dispatcher.utter_message(text="action_check_person_name")
        slot_value= tracker.get_slot("person_name")
        if slot_value!=None:
           return [SlotSet("person_name_known", True)]
        else:
           #dispatcher.utter_message(text="Name is unknown, python!")
           return [SlotSet("person_name_known", False)]

class ActionCheckPersonDrink(Action):
    def name(self) -> Text:
        return "action_check_person_drink"

    def run(self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        #dispatcher.utter_message(text="action_check_person_drink")
        slot_value = tracker.get_slot("person_drink")
        if slot_value!=None:
           #dispatcher.utter_message(text="Name is known, python!")

           return [SlotSet("person_drink_known", True)]
        else:
           #dispatcher.utter_message(text="Name is unknown, python!")
           return [SlotSet("person_drink_known", False)]

class ActionSlotReset(Action):
    def name(self):
        return 'action_slot_reset'
    def run(self, dispatcher, tracker, domain):
        return[AllSlotsReset()]

class ActionDefaultFallback(Action):
    """Executes the fallback action and goes back to the previous state
    of the dialogue"""

    def name(self) -> Text:
        return "action_default_fallback"

    async def run(
        self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        dispatcher.utter_message(text="I'm sorry, I didn't quite understand that")
        
        rospy.init_node('rasa_actions', anonymous=True)	
        pub_bool = rospy.Publisher("rasa_confirmation", Bool, queue_size=1)
        rospy.sleep(1)
        pub_bool.publish(False)
        rospy.sleep(1)
        '''
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    
        cli_args = ["/home/lcastor/ros_ws/src/LCASTOR/lcastor_nlu/robocup_nlu/launch/bool_publish.launch",'topic_name:=rasa_confirmation','value:=False']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        '''
        
        # Revert user message which led to fallback.
        return [UserUtteranceReverted()]

"""
class planner_class:
    def __init__(self): #It is done only the first iteration
        self.intention="none"
    
    def planner_intention_callback(self,msg):
        intention=msg.data
        self.intention=intention

pub_hz=0.01 #main loop frequency
# Main Script
if __name__ == '__main__':   
    planner=planner_class()
    # Initialize our node    
    rospy.init_node('han_velocity_control',anonymous=True)
    rospy.Subscriber('planner_intention',String,planner.intention_callback)  
    #Rate setup
    rate = rospy.Rate(1/pub_hz)  # main loop frecuency in Hz
    while not rospy.is_shutdown():	 
        #Publish message to let twist_mux know about the han priority
        rospy.loginfo("HOLA")
        print("HAN_ACTION",planner.intention)
        rate.sleep() #to keep fixed the publishing loop rate



class ValidateObjectToGraspForm(FormValidationAction):
    def name(self) -> Text:
        return "validate_object_to_grasp_form"
    
    @staticmethod
    def object_name_db() -> List[Text]:
        #Database of supported objects

        return ["cup","bottle", "orange","book","pen","box","remote","bowl","chair","cell phone","banana","apple"]
        
    def validate_object_name(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        #Validate value
        print("Slot value for object_name:",slot_value)
        if slot_value.lower() not in self.object_name_db():
            dispatcher.utter_message(text="I'm not familiar with this object!")
            return {"object_name": None}
        #dispatcher.utter_message(text="Ok, you chosen %s" % slot_value)
        return {"object_name": slot_value}

    
class ValidatePersonDescriptionForm(FormValidationAction):
    def name(self) -> Text:
        return "validate_person_description_form"
        
    @staticmethod
    def clothes_type_db() -> List[Text]:
        #Database of supported clothes

        return ["shirt", "pants", "jumper","sweater","shoes","hat","jacket"]
        
    @staticmethod
    def clothes_color_db() -> List[Text]:
        #Database of supported clothes colors

        return ["red", "blue", "black","green","white","magenta","orange"]
        
    def validate_clothes_type(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        #Validate value.
        print("Slot value for clothes_type:",slot_value)
        if slot_value.lower() not in self.clothes_type_db():
            dispatcher.utter_message(text=f"I'm not familiar with this kind of clothes")
            return {"clothes_type": None}
        dispatcher.utter_message(text="Ok, you chosen %s" % slot_value)
        return {"clothes_type": slot_value}
    
    def validate_clothes_color(
        self,
        slot_value: Any,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: DomainDict,
    ) -> Dict[Text, Any]:
        #Validate value.
        print("Slot value for clothes_color:",slot_value)
        if slot_value.lower() not in self.clothes_color_db():
            dispatcher.utter_message(text=f"I'm not familiar with this color")
            return {"clothes_type": None,"clothes_color": None}
        dispatcher.utter_message(text=f"Ok, you said {slot_value} color")
        return {"clothes_color": slot_value}
"""     
 
"""   
if __name__ == '__main__':
    # Initialize our node       
    rospy.init_node('rasa_actions')	
    #pub_trigger = rospy.Publisher("/nlp/trigger", Bool, queue_size=1)
    pub_username = rospy.Publisher("/nlp/username", String, queue_size=1)
    #priority_msg = Bool()
    rate = rospy.Rate(1/0.01)  # main loop frecuency in Hz
    username="hola"
    print("USERNAME",username)
    while not rospy.is_shutdown():	
        #pub_trigger.publish(robot.trigger)
        pub_username.publish(username)
        rate.sleep() #to keep fixed the publishing loop rate
"""  

'''
class ActionGoTo(Action):
    def name(self) -> Text:
        return "action_go_to"

    def run(self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        location = tracker.get_slot("location")
        dispatcher.utter_message(text="I'm going to %s now!" % location)

        # write here the code to interact with the robot

        return []

class ActionPersonFollow(Action):
    def name(self) -> Text:
        return "action_person_follow"

    def run(self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        #clothes_type = tracker.get_slot("clothes_type")
        #clothes_color = tracker.get_slot("clothes_color")
        #if clothes_type!=None and clothes_color!=None:
        #    dispatcher.utter_message(text="Ok, following a person with %s %s" % (clothes_color,clothes_type))
        #else:
        #    dispatcher.utter_message(text="Following a person, python!")
        
        dispatcher.utter_message(text="Following you!")
             
        
        rospy.init_node('rasa_actions', anonymous=True)
        rospy.loginfo("ROS Person following started")
        	
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        cli_args = ["/home/lcastor/ros_ws/src/LCASTOR/lcastor_nlu/robocup_nlu/launch/trigger_publish.launch",'trigger:=True']
        #cli_args = ["/home/leo/catkin_ws/src/LCASTOR/lcastor_nlu/robocup_nlu/launch/trigger_publish.launch",'trigger:=True']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        """
        
        
        rospy.init_node('rasa_actions')	
        pub_trigger = rospy.Publisher("/nlp/trigger", Bool, queue_size=1)
        pub_trigger.publish(True)
        
        
        #import roslaunch
        #import rospy
        rospy.init_node('rasa_actions', anonymous=True)	
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        #launch_person_follow = roslaunch.parent.ROSLaunchParent(uuid, ["/home/leo/catkin_ws/src/robocup_human_sensing/launch/webcam.launch"])
        launch_person_follow = roslaunch.parent.ROSLaunchParent(uuid, ["/home/lcastor/ros_ws/src/LCASTOR/lcastor_nlu/robocup_nlu/launch/trigger_publish.launch"])
        launch_person_follow.start()

        
        
        launch_person_follow.start()
        rospy.loginfo("ROS Person following started")
        """

        # write here the code to interact with the robot

        return []
'''

'''        
class ActionGraspObject(Action):
    def name(self) -> Text:
        return "action_grasp_object"
        

    def run(self,
        dispatcher: CollectingDispatcher,
        tracker: Tracker,
        domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        object_name = tracker.get_slot("object_name")
        
        dispatcher.utter_message(text="Grasping a %s!" % object_name)
        
        # write here the code to interact with the robot

        return []
'''

  
