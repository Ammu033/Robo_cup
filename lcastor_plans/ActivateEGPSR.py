import os import sys
from ollamamessages.msg import WhisperTranscription, WhisperListening
from ollamamessages.srv import OllamaCall
from lcastor_grasping.srv import ObjectList, ObjectListRequest, ObjectListResponse
from lcastor_grasping.srv import ObjectFloorPose, ObjectFloorPoseRequest, ObjectFloorPoseResponse
from AskConfirmation import AskConfirmation

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import time
import pnp_cmd_ros
from pnp_cmd_ros import *
from std_msgs.msg import Bool, String
import rospy
import numpy as np
import math
import tf
from geometry_msgs.msg import PointStamped , PoseWithCovarianceStamped

ROOM_DICT_B = { 
    "hallwaycabinet" : [-2.93, -4.47, 0.0, 0.0, 0.99, -0.08],
    "hallway" : [-1.41, -2.64, 0.0, 0.0, 0.95, -0.30],
    "entrance" : [-1.41, -2.64, 0.0, 0.0, 0.95, -0.30],
    "desk" : [-2.79, -0.46, 0.0, 0.0, -0.78, 0.61],
    "office" : [-0.96, -1.21, 0.0, 0.0, 0.85, 0.52],
    "studio" : [-0.96, -1.21, 0.0, 0.0, 0.85, 0.52],
    "shelf" : [-3.03, 1.55, 0.0, 0.0, 0.99, 0.03],
    "coathanger" : [-1.73, -2.89, 0.0, 0.0, 0.89, 0.44],
    "exit" : [-0.99, 1.61, 0.0, 0.0, 0.67, 0.73],
    "TVtable" : [1.01, -4.53, 0.0, 0.0, 0.99, -0.05],
    "loungechair" : [1.64, -4.85, 0.0, 0.0, -0.09, 0.99],
    "lamp" : [3.26, -5.12, 0.0, 0.0, -0.12, 0.99],
    "couch" : [3.6, -2.60, 0.0, 0.0, -0.35, 0.93],
    "coffetable" : [2.45, -3.20, 0.0, 0.0, -0.55, 0.83],
    "lounge" : [2.72, -1.96, 0.0, 0.0, -0.67, 0.73],
    "livingroom" : [2.72, -1.96, 0.0, 0.0, -0.67, 0.73],
    "trashcan" : [0.58, -1.16, 0.0, 0.0, 0.98, -0.17],
    "kitchen" : [3.34, -1.76, 0.0, 0.0, 0.84, 0.54],
    "kitchencabinet" : [0.62, 2.29, 0.0, 0.0, 0.99, 0.03],
    "dinnertable" : [1.44, 1.28, 0.0, 0.0, -0.02, 0.99],
    "dishwasher" : [3.67, 0.73, 0.0, 0.0, 0.04, 0.99],
    "kitchencounter" : [3.80, 1.98, 0.0, 0.0, 0.-0.0, 0.99],
    "inspectionpoint" : [0.19, -2.69, 0.0, 0.0, -0.48, 0.87],
    "findTrashEntrance" : [-0.61, 5.89, 0.0, 0.0, 0.05, 0.99],
    "findTrashOffice" : [0.30, 4.63, 0.0 , 0.0, 0.91, -0.41],
    "findTrashKitchen1" : [-3.18, 7.15, 0.0, 0.0, 0.97, -0.24],
    "findTrashKitchen2" : [-5.74, 10.44, 0.0, 0.0, -0.82, 0.57],
    "findTrashLivingRoom" : [-5.74, 10.40, 0.0, 0.0, -0.07, 0.99],
}

LOCATIONS = list(ROOM_DICT_B.keys())

POSSIBLE_PEOPLE_AREAS = [
    'findTrashEntrance',
    'findTrashOffice',
    'findTrashKitchen2',
    'findTrashLivingRoom',
]

POSSIBLE_TRASH_AREAS = [
    'findTrashEntrance', 
    'findTrashOffice', 
    'findTrashKitchen1',
    'findTrashKitchen2',
    'findTrashLivingRoom',
    'inspectionpoint',
]

OBJECT_CATEGORY = {
    "soap":  "cleaning_supplies",
    "dishwasher_tab":  "cleaning_supplies",
    "washcloth":  "cleaning_supplies",
    "sponge":  "cleaning_supplies",
    "cola":  "drinks",
    "ice_tea":  "drinks",
    "water":  "drinks",
    "milk":  "drinks",
    "big_coke":  "drinks",
    "fanta":  "drinks",
    "dubbelfris":  "drinks",
    "cornflakes":  "food",
    "pea_soup":  "food",
    "curry":  "food",
    "pancake_mix":  "food",
    "hagelslag":  "food",
    "sausages":  "food",
    "mayonaise":  "food",
    "candle":  "decorations",
    "pear":  "fruits",
    "plum":  "fruits",
    "peach":  "fruits",
    "lemon":  "fruits",
    "orange":  "fruits",
    "strawberry":  "fruits",
    "banana":  "fruits",
    "apple":  "fruits",
    "stroopwafel":  "snacks",
    "candy":  "snacks",
    "liquorice":  "snacks",
    "crisps":  "snacks",
    "pringles":  "snacks",
    "tictac":  "snacks",
    "spoon":  "dishes",
    "plate":  "dishes",
    "cup":  "dishes",
    "fork":  "dishes",
    "bowl":  "dishes",
    "knife":  "dishes",
}

CATRGORY_LOCATION = {
    'desk': 'decorations',
    'shelf': 'cleaning_supplies',
    'TVtable': 'toys',
    'coffetable': 'fruits',
    'kitchencabinet': 'drinks',
    'dinnertable': 'snacks',
    'dishwasher': 'dishes',
    'kitchencounter': 'food',
}

# whisper situations
WAITING = 0
READY_FOR_QUEST = 1

class EGPSR:
    def __init__(self, p):
        self.p = p
        self.pause = 0.8
        self.energy = 4000
        self.dynamic_energy = False
        self.no_speech_thresh = 0.2
        self.time_open_gripper = 4 

        rospy.set_param("/stt/use_ollama", False)
        rospy.set_param("/stt/speech_recogn_pause_time", self.pause)
        rospy.set_param("/stt/speech_recogn_energy", self.energy) 
        rospy.set_param("/stt/speech_recogn_dyn_energy_flag", self.dynamic_energy)
        rospy.set_param("/stt/speech_confidence_thresh", self.no_speech_thresh)
        rospy.loginfo(f'GPSR PARAMS - Pause: {self.pause}, Energy: {self.energy}, Dynamic Energy: {self.dynamic_energy}, No Speech Threshold: {self.no_speech_thresh}')
        self.listening_pub = rospy.Publisher("/stt/listening", WhisperListening, queue_size=1)

    def call_whisper(self, situation: int, tries=0):
        '''
        sends a message over to whisper to recieve speech input
        '''
        self.listening_pub.publish(listening=False)
        
        if situation == WAITING:
            time.sleep(4)
            p.exec_action('speak', 'please_say,_im_ready,_when_you_want_to_start')
            time.sleep(1)
        if situation == READY_FOR_QUEST:
            rospy.logerr('Cannot request for a new quest/cmd while already doing one')
            if tries == 0: 
                p.exec_action('speak', 'Hi_how_can_I_help_you_today')
            else:
                p.exec_action('speak', 'sorry,_could_you_repeat_that_more_closely')

        self.listening_pub.publish(listening=True)
        whisper_response = rospy.wait_for_message('/stt/transcription', WhisperTranscription)
        self.listening_pub.publish(listening=False)

        if whisper_response.no_speech_prob <= self.no_speech_thresh:
            return True, whisper_response.text
        return False, whisper_response.text   

    def obtain_quest_from_person(self):
        time.sleep(3)
        success, quest_speech = self.call_whisper(READY_FOR_QUEST)
        try:
            service_call = rospy.ServiceProxy("/gpsr/task_decomposition", OllamaCall)
            response = service_call(input = quest_speech)
            self.listening_pub.publish(listening = False)
            print(response)
        except Exception as e:
            print("GPSR failed: ", str(e))
            p.exec_action('speak', 'Sorry,_the_tasks_might_have_been_difficult_to_understand')
        rospy.loginfo("Successfully sent, generating GPSR, stopping listening.")

    def open_door(self) -> None:
        self.p.exec_action("moveHead", "0.0_0.0")
        self.p.exec_action("speak", "Can_you_please_open_the_door_for_me_?")
        while(not self.p.get_condition("isDoorOpen")): time.sleep(0.1)
        self.p.exec_action("speak", "Thank_you_for_opening_the_door")
        self.p.exec_action("navigateForward", "7")

    def find_waving_people(self) -> bool:
        '''
        1- check if there are waving people
        2- if so find their location
        3- goto the person

        Return:
            bool: if person spotted and we have arrived at their location
        '''
        start_time = rospy.Time().now().secs
        found_person = False
        reached_person = False
        while rospy.Time.now().secs - start_time < 15.0:
            if rospy.get_param('/found_person' , False):
                found_person = True
                break
        if found_person :
            guest_location = [rospy.get_param('/person_location/x'),
                    rospy.get_param('/person_location/y'),
                    rospy.get_param('/person_location/z')] 
        # gotoPerson(guest_location)
            p.exec_action('gotoPerson' , str(guest_location[0]) + '_' + str(guest_location[1]) + '_' + str(guest_location[2])   )
            if rospy.get_param('reached_person' , False) :
                reached_person = True
        return reached_person and found_person

    def phase_look_for_people(self, max_people) -> None:
        ''' THERE SHOULD ONLY BE 2 PEOPLE WITH TASKS'''
        # 1. go sensible people locations (outer while loop or 2 people found)
        try: 
            remember_location = []
            total_quests = 0
            for location in POSSIBLE_PEOPLE_AREAS:
                if total_quests <= max_people:
                    # 2. scan the locations for people waving
                    spotted = self.find_waving_people()
                    # 3. if person spotted, go speak to them and execute GPSR
                    if spotted and (location not in remember_location):
                        remember_location.append(location)
                        self.obtain_quest_from_person()
                        total_quests += 1
        except Exception as e:
            rospy.logerr(e)

    def phase_look_for_trash(self):
        for location in POSSIBLE_TRASH_AREAS:
            try:
                self.p.exec_action('gotoRoom', 'r_'+location)
                self.p.exec_action('speak', 'checking_'+location+'_for_misplaced_items')
                self.p.exec_action('moveHead', '0.0_-0.75')
                self.p.exec_action('moveTorso', '0.35')
                self.scan_location_trash()
            except Exception as e:
                self.p.exec_action('moveHead', '0.0_0.0')
                rospy.logerr(e)

    def scan_location_trash(self):
        try:
            # find all trash 
            req = ObjectFloorPoseRequest()
            req.z_cutoff = 0.5

            detect_service_call = rospy.ServiceProxy("get_object_floor_poses", ObjectFloorPose)
            detect_service_call.wait_for_service(timeout=3.0) # wait for service to be available

            trash = detect_service_call(req)

            for object in trash:
                self.send_to_trash(object)

        except rospy.ServiceException as e:
            rospy.logerr(e)
        except Exception as e:
            rospy.logerr(e)
        

    def send_to_trash(self, object_poses):
        # object poses in the map frame 
        global goal_msg, robot_pose, person_point
        # Obtain the current robot pose
        robot_pose_data  = rospy.wait_for_message('/robot_pose' , PoseWithCovarianceStamped )
        q = (
                    robot_pose_data.pose.pose.orientation.x,
                    robot_pose_data.pose.pose.orientation.y,
                    robot_pose_data.pose.pose.orientation.z,
                    robot_pose_data.pose.pose.orientation.w
                )
        m = tf.transformations.quaternion_matrix(q)
        robot_pose_x = robot_pose_data.pose.pose.position.x
        robot_pose_y = robot_pose_data.pose.pose.position.y
        robot_pose_theta = tf.transformations.euler_from_matrix(m)[2]

        # Compute distance in cartesian space between obj and robot
        obj_pos = np.array([object_poses.point.x , object_poses.point.y])
        robot_pos = np.array([robot_pose_x , robot_pose_y])
        vector_to_obj = obj_pos - robot_pos
        distance_to_obj = math.sqrt(vector_to_obj[0]**2 + vector_to_obj[1]**2)
        # Normalize the vector to the desired distance
        normalized_vector = vector_to_obj / distance_to_obj
        # Calculate the orientation needed to reach the obj
        goal_orientation = math.atan2(normalized_vector[1], normalized_vector[0])
        # Calculate the goal position based on the desired distance
        goal_position = obj_pos - DES_DIST * normalized_vector
        self.p.exec_action('goto', str(goal_position[0]) + "_" + str(goal_position[1]) + '_' + str(goal_orientation))
        self.p.exec_action('speak', 'Please_help_me_pick_up_the_trash,_Place_it_into_my_gripper')
        self.p.exec_action("gripperAction", "open")
        time.sleep(self.time_open_gripper)
        self.p.exec_action("gripperAction", "close")
        time.sleep(3)
        self.p.exec_action('speech', 'thank_you')
        self.p.exec_action("gotoRoom", "r_trashcan")

        self.p.exec_action('speak', 'Please_help_me_removing_the_trash_from_my_hand')
        self.p.exec_action("gripperAction", "open")
        time.sleep(self.time_open_gripper)
        self.p.exec_action("gripperAction", "close")
        time.sleep(3)
        self.p.exec_action('speech', 'thank_you')



    def phase_look_for_incorrectly_placed_objects(self):
        head_tilt = '0.0_-0.8'
        for location in CATRGORY_LOCATION.keys():
            try: 
                self.p.exec_action('gotoRoom', 'r_'+location)
                self.p.exec_action('speak', 'checking_'+location+'_for_misplaced_items')
                self.p.exec_action('moveHead', head_tilt)
                self.scan_location_objects(location)
                self.p.exec_action('moveHead', '0.0_0.0')
            except Exception as e:
                rospy.logerr(e)
                rospy.loginfo(location +' failed, moving onto the next location' )


    def object_location(self, object):
        location = 'unknown'
        try:
            if object in OBJECT_CATEGORY.keys():
                category = OBJECT_CATEGORY[object]
                location = CATRGORY_LOCATION[category]
        except Exception as e:
            rospy.logerr(e)
            location = 'unknown'
        return location


    def scan_location_objects(self, location):
        detected_objects = None

        try:
            # find all objects
            req = ObjectListRequest()

            detect_service_call = rospy.ServiceProxy("/get_object_list", ObjectList)
            detect_service_call.wait_for_service(timeout=3.0) # wait for service to be available
            detected_objects = detect_service_call(req)
            print(detected_objects)
            move_to_correct_location = False
            try: 
                if detected_objects and (len(detected_objects) == 1):
                    move_to_correct_location = True
            except Exception as e:
                rospy.logerr(e)

            for object_msg in detected_objects:
                object = object_msg.data
                correct_location = self.object_location(object)
                if correct_location == 'unknown':
                   continue 
                if correct_location != location: 
                    self.object_in_wrong_location(object, location, move_to_correct_location)
                else:
                    rospy.loginfo('Seems like there are no objects to move here')
        except rospy.ServiceException as e:
            rospy.logerr(e)
        except Exception as e:
            rospy.logerr(e)

    def object_in_wrong_location(self, object, location, move = False):
        self.p.exec_action('speak', object+'_is_incorrectly_placed,_it_should_be_in_'+location)

        if move:
        # for more points, do we want to try grabbing the objects?
            self.p.exec_action('speak', 'please_help_place_'+object+'in_my_hand')
            self.p.exec_action("gripperAction", "open")
            time.sleep(self.time_open_gripper)
            self.p.exec_action("gripperAction", "close")
            time.sleep(1)
            self.p.exec_action('speak', 'thank_you')
            time.sleep(3)
            self.p.exec_action("gotoRoom", "r_"+location)
            time.sleep(3)
            self.p.exec_action('speak', 'please_help_remove_'+object+'in_my_hand')
            self.p.exec_action("gripperAction", "open")
            time.sleep(self.time_open_gripper)
            self.p.exec_action("gripperAction", "close")
            self.p.exec_action('speak', 'thank_you')

    
    def start(self):
        # 1. Wait for the door open
        self.open_door()
        
        self.p.exec_action('moveHead', '0.0_0.0')
        self.p.exec_action('moveTorso', '0.0_0.0')

        self.phase_look_for_incorrectly_placed_objects()
         
        self.p.exec_action('moveHead', '0.0_0.0')
        self.p.exec_action('moveTorso', '0.0_0.0')

        self.phase_look_for_trash()
         
        self.p.exec_action('moveHead', '0.0_0.0')
        self.p.exec_action('moveTorso', '0.0_0.0')

        self.phase_look_for_people(max_people = 2)

if __name__ == "__main__":
    p = PNPCmd()
    p.begin()
    gpsr = EGPSR(p)
    gpsr.start()
    p.end()
