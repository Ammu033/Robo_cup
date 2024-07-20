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

from gotoRoom import ROOM_DICT_B
import time
import pnp_cmd_ros
from pnp_cmd_ros import *
from std_msgs.msg import Bool, String
import rospy
import numpy as np
import math
import tf
from geometry_msgs.msg import PointStamped , PoseWithCovarianceStamped

DES_DIST = 1.2
l = None
rotation_angle = None
person_point = None
robot_pose = None
goal_msg = None
client = None
goal_tolerance_client = None
gotopersonDone = False

LOCATIONS = list(ROOM_DICT_B.keys())

#TODO: these locations needs to be set and configured for this task - ricardo
POSSIBLE_PEOPLE_AREAS ['fill','sensible','locations','...']
POSSIBLE_TRASH_AREAS = ['trash_loc_1', 'trash_loc_2', '...']

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
        self.current_goal_location = LOCATIONS[0]
        self.previous_goal_location = None
        self.person_spotted = True
        self.continue_looking = True
        self.on_quest = False

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
            if self.on_quest:
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
        self.on_quest = True

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
        #TODO: Hari
        raise NotImplemented

    def phase_look_for_people(self) -> None:
        ''' THERE SHOULD ONLY BE 2 PEOPLE WITH TASKS'''
        # 1. go sensible people locations (outer while loop or 2 people found)
        remember_location = []
        total_quests = 0
        for location in POSSIBLE_PEOPLE_AREAS:
            if total_quests >=2:
                # 2. scan the locations for people waving
                spotted = self.find_waving_people()
                # 3. if person spotted, go speak to them and execute GPSR
                if spotted and (location not in remember_location):
                    remember_location.append(location)
                    self.obtain_quest_from_person()
                    total_quests += 1

    def phase_look_for_trash(self):
        torso_height = '0.0' #TODO: set
        head_tilt = '0.0_-0.4' #TODO: we need a better head tilt
        for location in POSSIBLE_TRASH_AREAS:
            self.p.exec_action('gotoRoom', 'r_'+location)
            self.p.exec_action('speak', 'checking_'+location+'_for_misplaced_items')
            self.p.exec_action('moveHead', head_tilt)
            self.scan_location_trash()
        raise NotImplemented

    def scan_location_trash(self):
        

        # find all trash 
        trash_poses = ObjectFloorPoseResponse()
        req = ObjectFloorPoseRequest()
        req.z_cutoff = 0.5

        detect_service_call = rospy.ServiceProxy("object_floor_pose", ObjectFloorPose)
        try:
            trash = detect_service_call(req)
            for object in trash:
                self.send_to_trash(object)
        except rospy.ServiceException as e:
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
        robot_pose.x = robot_pose_data.pose.pose.position.x
        robot_pose.y = robot_pose_data.pose.pose.position.y
        robot_pose.theta = tf.transformations.euler_from_matrix(m)[2]
        # Compute distance in cartesian space between obj and robot
        obj_pos = np.array([object_poses.point.x , object_poses.point.y])
        robot_pos = np.array([robot_pose.x , robot_pose.y])
        vector_to_obj = obj_pos - robot_pos
        distance_to_obj = math.sqrt(vector_to_obj[0]**2 + vector_to_obj[1]**2)
        # Normalize the vector to the desired distance
        normalized_vector = vector_to_obj / distance_to_obj
        # Calculate the orientation needed to reach the obj
        goal_orientation = math.atan2(normalized_vector[1], normalized_vector[0])
        # Calculate the goal position based on the desired distance
        goal_position = obj_pos - DES_DIST * normalized_vector
        p.execAction('goto', str(goal_position[0]) + "_" + str(goal_position[1]) + '_' + str(goal_orientation))

    def phase_look_for_incorrectly_placed_objects(self):
        head_tilt = '0.0_-0.8'
        torso_height = '0.0'
        for location in CATRGORY_LOCATION.keys():
            self.p.exec_action('gotoRoom', 'r_'+location)
            self.p.exec_action('speak', 'checking_'+location+'_for_misplaced_items')
            self.p.exec_action('moveHead', head_tilt)
            self.scan_location_objects(location)
            self.p.exec_action('moveHead', '0.0_0.0')

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

            for object_msg in detected_objects:
                object = object_msg.data
                correct_location = self.object_location(object)
                if correct_location == 'unknown':
                   continue 
                if correct_location != location: 
                    self.object_in_wrong_location(object)
                else:
                    rospy.loginfo('Seems like there are no objects to move here')
        except rospy.ServiceException as e:
            rospy.logerr(e)
        except Exception as e:
            rospy.logerr(e)

    def object_in_wrong_location(self, object):
        self.p.exec_action('speak', object+'_is_incorrectly_placed,_it_should_be_in_'+ self.object_location(object))
        # for more points, do we want to try grabbing the objects?
    
    def start(self):
        # 1. Wait for the door open
        self.open_door()
    
        # Phase 1 - get quests from poeple 
        self.phase_look_for_people()

        # Phase 2 - scan the ground to look for trash


        # Phase 3 - scan the scannable locations for incorrectly placed items




if __name__ == "__main__":
    p = PNPCmd()
    p.begin()
    gpsr = EGPSR(p)
    gpsr.start()
    # gpsr.obtain_quest_from_person()
    p.end()
