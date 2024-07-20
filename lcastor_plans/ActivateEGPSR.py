import os import sys
from ollamamessages.msg import WhisperTranscription, WhisperListening
from ollamamessages.srv import OllamaCall
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

LOCATIONS = list(ROOM_DICT_B.keys())

#TODO: these locations needs to be set and configured for this task - ricardo
ROOM_LOCATION_CYCLE = ['core_locs1','...']
POSSIBLE_PEOPLE_AREAS ['fill','sensible','locations','...']
POSSIBLE_OBJECT_AREAS = ['coffetable', 'table', '...']
POSSIBLE_TRASH_AREAS = ['trash_loc_1', 'trash_loc_2', '...']

OBJECT_LOCATIONS = {
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
                p.exec_action('speak', 'I_am_ready_for_my_quest')
            else:
                p.exec_action('speak', 'sorry,_could_you_repeat_that_more_closely')

        self.listening_pub.publish(listening=True)
        whisper_response = rospy.wait_for_message('/stt/transcription', WhisperTranscription)
        self.listening_pub.publish(listening=False)

        if whisper_response.no_speech_prob <= self.no_speech_thresh:
            return True, whisper_response.text
        return False, whisper_response.text   

    def obtain_quest_from_person(self):
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

    # # @property
    # def get_new_location(self):
    #     idx_loc = ROOM_LOCATION_CYCLE.index(self.current_goal_location)
    #     new_idx = idx_loc + 1
    #     if new_idx > len(ROOM_LOCATION_CYCLE):
    #         new_idx = 0
    #     ROOM_LOCATION_CYCLE[new_idx]

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
        head_tilt = '0.0_0.0' #TODO: we need a better head tilt
        for location in POSSIBLE_TRASH_AREAS:
            self.p.exec_action('gotoRoom', 'r_'+location)
            self.p.exec_action('speak', 'checking_'+location+'_for_misplaced_items')
            self.p.exec_action('moveHead', head_tilt)
            self.scan_location_trash()
        raise NotImplemented

    def scan_location_trash(self):
        # find all trash 
        # TODO: Niko to find the trash in the image frame -> returns a list trash -> array
        # cut off objects above a certain z height 
        detect_service_call = rospy.ServiceProxy("object_floor_pose", ObjectFloorPose)
        try:
            trash = detect_service_call()
            for object in trash:
                self.send_to_trash(object)
        except rospy.ServiceException as e:
            rospy.logerr(e)
        

    def scan_location_trash_alt(self, location):
        # find all trash 
        # send a request back to ollama to capture the image now to check for trash
        # this would return speech saying what trash was foind in the image, thats as far as it goes
        raise NotImplemented
    

    def send_to_trash(self, object_poses):
        # object poses in the map frame 
        # ricardo francesco (sarah will give a pose stamp)
        #TODO: sarah - it's late I'm not that sure to be honest
        # basically we want to spot the trash and then either speak 
        # or try to pick it up and take it to the bin
        raise NotImplemented

    def phase_look_for_incorrectly_placed_objects(self):
        head_tilt = '0.0_0.0' #TODO: we need a better head tilt
        torso_height = '0.0' #TODO:
        # 1. Go to possible object locations
        for location in POSSIBLE_OBJECT_AREAS:
            self.p.exec_action('gotoRoom', 'r_'+location)
            self.p.exec_action('speak', 'checking_'+location+'_for_misplaced_items')
            self.p.exec_action('moveHead', head_tilt)
            self.scan_location_objects(location)
        raise NotImplemented
   

    def scan_location_objects(self, location):
        # find all objects
        # TODO: Niko to find the objects in the image frame -> returns a list objects -> array
        detect_service_call = rospy.ServiceProxy("/get_object_list", OllamaCall)
        response = service_call(input = quest_speech)
    
        req = ObjectList()
        req
        detect_service_call = rospy.ServiceProxy("detect_object_list", ObjectList)
            try:
                trash = detect_service_call()
                for object in trash:
                    self.send_to_trash(object)
            except rospy.ServiceException as e:
                rospy.logerr(e)
     
        objects = ['found', 'objects'] # <- example of the outputs from the scene
        objects = 

        for object in objects:
            if OBJECT_LOCATIONS[object] != location:
                self.object_in_wrong_location(object)
        raise NotImplemented

    def scan_location_objects_alt(self, location):
        # find all objects in the wrong place 
        # this would return speech saying what objects were foind in the image, thats as far as it goes
        raise NotImplemented
    

    def object_in_wrong_location(self, object):
        self.p.exec_action('speak', object+'_is_incorrectly_placed,_it_should_be_in_'+ OBJECT_LOCATIONS[object])
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
    p.end()
