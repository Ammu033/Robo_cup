import os 
import sys
from ollamamessages.msg import WhisperTranscription, WhisperListening
from ollamamessages.srv import OllamaCall
from lcastor_grasping.srv import ObjectList, ObjectListRequest, ObjectListResponse
from tiago_auto.srv import ObjectFloorPose, ObjectFloorPoseRequest, ObjectFloorPoseResponse
from AskConfirmation import AskConfirmation
import subprocess 
import threading
import time
import queue

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)
# sys.path.insert(1, os.path.join(os.path.dir(__file__), "..", "..", "lcastor_actions"))
# import gotoRoom

import time
import pnp_cmd_ros
from pnp_cmd_ros import *
from std_msgs.msg import Bool, String
import rospy
import numpy as np
import math
import tf
from geometry_msgs.msg import PointStamped , PoseWithCovarianceStamped

POSSIBLE_PEOPLE_AREAS = [
    "room_1",
    "room_2",
    "room_3",
    "room_4",
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
        self.continuous_scanning = False
        self.scanning_thread = None
        self.current_destination = None
        self.planned_locations = []
        self.location_index = 0
        self.navigation_interrupted = False
        self.trash_detected_event = threading.Event()
        self.current_trash_location = None
        self.trash_lock = threading.Lock()

        rospy.set_param("/stt/use_ollama", False)
        rospy.set_param("/stt/speech_recogn_pause_time", self.pause)
        rospy.set_param("/stt/speech_recogn_energy", self.energy) 
        rospy.set_param("/stt/speech_recogn_dyn_energy_flag", self.dynamic_energy)
        rospy.set_param("/stt/speech_confidence_thresh", self.no_speech_thresh)
        rospy.loginfo(f'GPSR PARAMS - Pause: {self.pause}, Energy: {self.energy}, Dynamic Energy: {self.dynamic_energy}, No Speech Threshold: {self.no_speech_thresh}')
        # self.listening_pub = rospy.Publisher("/stt/listening", WhisperListening, queue_size=1)

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
                p.exec_action('speak', 'I_detected_a_person,_please_come_to_me.')
                time.sleep(3)
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
        rospy.set_param('/found_person' , False)
        while rospy.Time.now().secs - start_time < 15.0:
            if rospy.get_param('/found_person' , False):
                found_person = True
                break
        if found_person :
            guest_location = [rospy.get_param('/person_location/x'),
                    rospy.get_param('/person_location/y'),
                    rospy.get_param('/person_location/z')] 
        # gotoPerson(guest_location)
            # self.goto_person(guest_location)
            # p.exec_action('gotoPerson' , str(guest_location[0]) + '_' + str(guest_location[1]) + '_' + str(guest_location[2])   )
            # if rospy.get_param('reached_person' , False) :
                # reached_person = True
        reached_person = True # FIXME: bypassing code above
        return reached_person and found_person
        # TODO: Hari
        # raise NotImplemented

    def goto_person(self, person_position_wrt_camera ):
        global goal_msg, robot_pose, person_point
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        #l = tf.TransformListener() 
        rospy.sleep(1.0)
        
        transform = tf_buffer.lookup_transform('map', 'xtion_depth_optical_frame', rospy.Time(0), rospy.Duration(1.0))
        
        
        rotation_angle = 0.0
        person_point = PointStamped()
        person_point.header.stamp = rospy.Time.now()
        person_point.header.frame_id = 'xtion_depth_optical_frame'
        robot_pose = Pose2D()
        goal_msg = MoveBaseGoal()
        client = actionlib.SimpleActionClient('/move_base' , MoveBaseAction)
        person_point.point.x = person_position_wrt_camera[0]
        person_point.point.y = person_position_wrt_camera[1]
        person_point.point.z = person_position_wrt_camera[2]
        print(person_position_wrt_camera)
        human_wrt_map = tf_buffer.transform(person_point,target_frame='map')
        #human_wrt_map = l.transformPoint(target_frame='map', ps=person_point)
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
        person_pos = np.array([human_wrt_map.point.x , human_wrt_map.point.y])
        print(person_pos)
        robot_pos = np.array([robot_pose.x , robot_pose.y])
        # print(person_pos)
        vector_to_person = person_pos - robot_pos

        # Calculate the distance from the robot to the person
        distance_to_person = math.sqrt(vector_to_person[0]**2 + vector_to_person[1]**2)

        # Normalize the vector to the desired distance
        normalized_vector = vector_to_person / distance_to_person

        # Calculate the orientation needed to reach the person
        goal_orientation = math.atan2(normalized_vector[1], normalized_vector[0])

        # Calculate the goal position based on the desired distance
        DES_DIST = 1.2
        goal_position = person_pos - DES_DIST * normalized_vector

        # return goal_position, goal_orientation    
        goal_msg.target_pose.header.stamp = rospy.Time.now()
        goal_msg.target_pose.header.frame_id = "map"
        goal_msg.target_pose.pose.position.x = goal_position[0]
        goal_msg.target_pose.pose.position.y = goal_position[1]
        print(goal_position)
        goal_msg.target_pose.pose.orientation.z = math.sin(goal_orientation/2)
        goal_msg.target_pose.pose.orientation.w = math.cos(goal_orientation/2)
        # config = goal_tolerance_client.update_configuration(people_config)
        # time.sleep(1.0)
        client.send_goal(goal_msg , done_cb=self.on_goto_done)
        while True:
            if gotopersonDone:
                break
    
    def on_goto_done(self, goalState,result):
        global gotopersonDone
        print(result)
        print('Go To Command Successfully Sent')
        rospy.set_param('reached_person' , True)
        gotopersonDone = True
        
    def phase_look_for_people(self) -> None:
        ''' THERE SHOULD ONLY BE 2 PEOPLE WITH TASKS'''
        # 1. go sensible people locations (outer while loop or 2 people found)
        try: 
            max_people = 2 # this killed us
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


    def phase_look_for_trash_with_interrupts(self):

        POSSIBLE_TRASH_AREAS = [
            "room_4", 
            "room_2",
            "room_3", 
            "room_1",
        ]
        
        self.planned_locations = POSSIBLE_TRASH_AREAS.copy()
        self.location_index = 0
        
        # Start interrupt-based scanning
        self.start_interrupt_trash_scanning()
        
        try:
            # Visit each planned location
            while self.location_index < len(self.planned_locations):
                location = self.planned_locations[self.location_index]
                
                rospy.loginfo(f"Phase: Going to {location} (Location {self.location_index + 1}/{len(self.planned_locations)})")
                
                # Navigate with interrupt monitoring
                self.navigate_to_location_with_interrupts(location)
                
                # Do detailed scan at destination
                rospy.loginfo(f"Arrived at {location}, performing detailed scan...")
                self.p.exec_action('moveHead', '0.0_-0.75')
                self.p.exec_action('moveTorso', '0.35')
                
                # Scan for any remaining trash at this location
                self.scan_location_trash()
                
                # Move to next location
                self.location_index += 1
                
                # Brief pause between locations
                time.sleep(2)
            
            rospy.loginfo("Completed all planned trash collection locations")
            
        except Exception as e:
            rospy.logerr(f"Error in interrupt-based trash phase: {e}")
        finally:
            # Stop interrupt scanning
            self.stop_interrupt_trash_scanning()
    
    def start_interrupt_trash_scanning(self):
      
        self.continuous_scanning = True
        self.scanning_thread = threading.Thread(target=self.interrupt_trash_scan_worker)
        self.scanning_thread.daemon = True
        self.scanning_thread.start()
        rospy.loginfo("Started interrupt-based trash scanning")
        
    def stop_interrupt_trash_scanning(self):
       
        self.continuous_scanning = False
        if self.scanning_thread:
            self.scanning_thread.join()
        rospy.loginfo("Stopped interrupt-based trash scanning")
    
    def interrupt_trash_scan_worker(self):
        
        scan_interval = 1.5  # Scan every 1.5 seconds for responsiveness
        
        while self.continuous_scanning:
            try:
                # Only scan if we're currently navigating
                if self.current_destination is not None:
                    req = ObjectFloorPoseRequest()
                    req.z_cutoff = 0.5
                    
                    detect_service_call = rospy.ServiceProxy("get_object_floor_poses", ObjectFloorPose)
                    trash_response = detect_service_call(req)
                    
                    if trash_response.floor_poses:
                        # Found trash! Trigger interrupt
                        closest_trash = self.find_closest_trash(trash_response.floor_poses)
                        
                        with self.trash_lock:
                            self.current_trash_location = closest_trash
                            self.navigation_interrupted = True
                            self.trash_detected_event.set()
                        
                        rospy.loginfo(f"INTERRUPT: Trash detected at ({closest_trash.x:.2f}, {closest_trash.y:.2f})!")
                        rospy.loginfo(f"Interrupting navigation to {self.current_destination}")
                        
                        # Wait until trash is collected before continuing to scan
                        while self.navigation_interrupted:
                            time.sleep(0.5)
                
                time.sleep(scan_interval)
                
            except Exception as e:
                rospy.logerr(f"Error in interrupt trash scanning: {e}")
                time.sleep(scan_interval)
            
    def find_closest_trash(self, trash_poses):
      
        try:
            robot_pose_data = rospy.wait_for_message('/robot_pose', PoseWithCovarianceStamped, timeout=1.0)
            robot_x = robot_pose_data.pose.pose.position.x
            robot_y = robot_pose_data.pose.pose.position.y
            
            closest_trash = None
            min_distance = float('inf')
            
            for trash_pose in trash_poses:
                distance = math.sqrt((trash_pose.x - robot_x)**2 + (trash_pose.y - robot_y)**2)
                if distance < min_distance:
                    min_distance = distance
                    closest_trash = trash_pose
            
            return closest_trash
            
        except Exception as e:
            rospy.logerr(f"Error finding closest trash: {e}")
            return trash_poses[0] if trash_poses else None
        
    def handle_trash_interrupt(self):
       
        with self.trash_lock:
            if self.current_trash_location:
                rospy.loginfo("Handling trash interrupt...")
                
                # Stop current navigation (if possible)
                self.stop_current_navigation()
                
                # Collect the trash
                rospy.loginfo("Collecting detected trash...")
                self.collect_trash_immediately(self.current_trash_location)
                
                # Go to bin
                rospy.loginfo("Going to bin to dispose trash...")
                if not self.goto_room_subprocess("Bin"):
                    rospy.logerr("Failed to navigate to bin")
                else:
                    # Dispose trash at bin
                    self.dispose_trash_at_bin()
                
                # Clear interrupt state
                self.current_trash_location = None
                self.navigation_interrupted = False
                self.trash_detected_event.clear()
                
                rospy.loginfo("Trash interrupt handled, resuming original plan...")
    
    def stop_current_navigation(self):
       
        try:
            # Send cancel goal to move_base if using actionlib
            # This is a simplified version - you might need to adapt based on your navigation system
            rospy.loginfo("Attempting to stop current navigation...")
            # subprocess.run(['rosnode', 'kill', '/move_base'], check=False)
            time.sleep(1)  # Brief pause to ensure navigation stops
        except Exception as e:
            rospy.logerr(f"Error stopping navigation: {e}")
    
    def collect_trash_immediately(self, trash_pose):
      
        try:
            # Get current robot position
            robot_pose_data = rospy.wait_for_message('/robot_pose', PoseWithCovarianceStamped, timeout=2.0)
            robot_x = robot_pose_data.pose.pose.position.x
            robot_y = robot_pose_data.pose.pose.position.y
            
            # Calculate vector from robot to trash
            trash_pos = np.array([trash_pose.x, trash_pose.y])
            robot_pos = np.array([robot_x, robot_y])
            vector_to_trash = trash_pos - robot_pos
            
            # Calculate orientation to face the trash
            orientation_to_trash = math.atan2(vector_to_trash[1], vector_to_trash[0])
            
            # Calculate approach position (stay a bit away from trash)
            distance_to_trash = math.sqrt(vector_to_trash[0]**2 + vector_to_trash[1]**2)
            APPROACH_DISTANCE = 0.8  # meters
            
            if distance_to_trash > APPROACH_DISTANCE:
                # Move closer to trash while facing it
                normalized_vector = vector_to_trash / distance_to_trash
                approach_pos = trash_pos - APPROACH_DISTANCE * normalized_vector
                approach_x, approach_y = approach_pos[0], approach_pos[1]
            else:
                # Already close enough, just turn to face it
                approach_x, approach_y = robot_x, robot_y
            
            rospy.loginfo(f"Moving to approach trash at ({approach_x:.2f}, {approach_y:.2f}) facing angle {orientation_to_trash:.2f}")
            
            # Navigate to approach position facing the trash
            if not self.goto_coordinates_subprocess(approach_x, approach_y, orientation_to_trash):
                rospy.logerr("Failed to navigate to trash approach position")
                return
            
            # Position robot for pickup
            self.p.exec_action('moveHead', '0.0_-0.75')
            self.p.exec_action('moveTorso', '0.35')
            
            # Collect trash
            self.p.exec_action('speak', 'I_found_trash._Please_help_me_pick_it_up')
            self.p.exec_action("gripperAction", "open")
            time.sleep(self.time_open_gripper)
            self.p.exec_action("gripperAction", "close")
            time.sleep(2)
            self.p.exec_action('speak', 'thank_you')
            
        except Exception as e:
            rospy.logerr(f"Error collecting trash: {e}")
    
    def dispose_trash_at_bin(self):
        
        try:
            #self.p.exec_action('speak', 'Please_help_me_dispose_this_trash')
            self.p.exec_action("gripperAction", "open")
            time.sleep(self.time_open_gripper)
            self.p.exec_action("gripperAction", "close")
            time.sleep(2)
            #self.p.exec_action('speak', 'thank_you')
            
        except Exception as e:
            rospy.logerr(f"Error disposing trash: {e}")
    
    def navigate_to_location_with_interrupts(self, location):
        
        self.current_destination = location
        rospy.loginfo(f"Starting navigation to {location} with interrupt monitoring...")
        
        # Start the navigation in a separate thread so we can monitor for interrupts
        nav_thread = threading.Thread(target=self.navigate_to_location_thread, args=(location,))
        nav_thread.daemon = True
        nav_thread.start()
        
        # Monitor for trash interrupts
        while nav_thread.is_alive():
            if self.trash_detected_event.wait(timeout=1.0):  # Check every second
                rospy.loginfo("Trash interrupt detected during navigation!")
                
                # Handle the interrupt
                self.handle_trash_interrupt()
                
                # Resume navigation to original destination
                rospy.loginfo(f"Resuming navigation to {location}")
                nav_thread = threading.Thread(target=self.navigate_to_location_thread, args=(location,))
                nav_thread.daemon = True
                nav_thread.start()
        
        # Navigation completed
        self.current_destination = None
        rospy.loginfo(f"Successfully reached {location}")
    
    def navigate_to_location_thread(self, location):
        """Thread worker for navigation"""
        try:
            self.goto_room_subprocess(location)
        except Exception as e:
            rospy.logerr(f"Navigation thread error: {e}")
                
    def scan_location_trash(self):
        try:
            # find all trash 
            req = ObjectFloorPoseRequest()
            req.z_cutoff = 0.5

            detect_service_call = rospy.ServiceProxy("get_object_floor_poses", ObjectFloorPose)
            detect_service_call.wait_for_service(timeout=3.0) # wait for service to be available

            trash_response = detect_service_call(req)
            
            if not trash_response.floor_poses:
                rospy.loginfo("No trash detected at this location")
                return
            
            # Iterate through detected trash objects
            for floor_pose in trash_response.floor_poses:
                rospy.loginfo(f"Found trash at position ({floor_pose.x:.2f}, {floor_pose.y:.2f}, {floor_pose.z:.2f})")
                
                # Send robot to pick up this trash
                self.send_to_trash(floor_pose)

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        except Exception as e:
            rospy.logerr(f"Error in scan_location_trash: {e}")
            
    def send_to_trash(self, object_pose):
        """
        using subprocess navigation
        """
        DES_DIST = 0.8
        
        try:
            # Obtain the current robot pose
            robot_pose_data = rospy.wait_for_message('/robot_pose', PoseWithCovarianceStamped, timeout=2.0)
            q = (
                robot_pose_data.pose.pose.orientation.x,
                robot_pose_data.pose.pose.orientation.y,
                robot_pose_data.pose.pose.orientation.z,
                robot_pose_data.pose.pose.orientation.w
            )
            m = tf.transformations.quaternion_matrix(q)
            robot_pose_x = robot_pose_data.pose.pose.position.x
            robot_pose_y = robot_pose_data.pose.pose.position.y

            # Compute goal position
            obj_pos = np.array([object_pose.x, object_pose.y])
            robot_pos = np.array([robot_pose_x, robot_pose_y])
            vector_to_obj = obj_pos - robot_pos
            distance_to_obj = math.sqrt(vector_to_obj[0]**2 + vector_to_obj[1]**2)
            
            if distance_to_obj == 0.8 :
                rospy.logwarn("Object is quite close to robot position, skipping")
                return
                
            normalized_vector = vector_to_obj / distance_to_obj
            goal_orientation = math.atan2(normalized_vector[1], normalized_vector[0])
            goal_position = obj_pos - DES_DIST * normalized_vector
            
            # Navigate to the object using subprocess
            if not self.goto_coordinates_subprocess(goal_position[0], goal_position[1], goal_orientation):
                rospy.logerr("Failed to navigate to trash object")
                return
            
            # Ask for help to pick up trash
            #self.p.exec_action('speak', 'I_found_trash_here._Please_help_me_pick_it_up_and_place_it_in_my_gripper')
            self.p.exec_action("gripperAction", "open")
            time.sleep(self.time_open_gripper)
            self.p.exec_action("gripperAction", "close")
            time.sleep(3)
            #self.p.exec_action('speak', 'thank_you')
            
            # Go to trash can using subprocess
            if not self.goto_room_subprocess("Bin"):
                rospy.logerr("Failed to navigate to trash can")
                return
            
            # Ask for help to dispose trash
            #self.p.exec_action('speak', 'Please_help_me_remove_the_trash_from_my_gripper')
            self.p.exec_action("gripperAction", "open")
            time.sleep(self.time_open_gripper)
            self.p.exec_action("gripperAction", "close")
            time.sleep(3)
            self.p.exec_action('speak', 'thank_you')
            
        except Exception as e:
            rospy.logerr(f"Error in send_to_bin: {e}")

    def goto_room_subprocess(self, room_name):
        """
        Navigate to room using subprocess call
        """
        try:
            rospy.loginfo(f"Navigating to room: {room_name}")
            result = subprocess.run([
                'rosrun', 'tiago_auto', 'goto_.py', 'room', room_name
            ], check=True, capture_output=True, text=True, timeout=120)
            
            rospy.loginfo(f"Successfully reached room: {room_name}")
            return True
            
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Navigation to {room_name} failed: {e}")
            return False
        except subprocess.TimeoutExpired:
            rospy.logerr(f"Navigation to {room_name} timed out")
            return False
    
    def goto_coordinates_subprocess(self, x, y, theta):
        """
        Navigate to specific coordinates using subprocess call
        """
        try:
            rospy.loginfo(f"Navigating to coordinates: x={x}, y={y}, theta={theta}")
            result = subprocess.run([
                'rosrun', 'tiago_auto', 'goto_.py', 'coordinates', 
                str(x), str(y), str(theta)
            ], check=True, capture_output=True, text=True, timeout=120)
            
            rospy.loginfo(f"Successfully reached coordinates")
            return True
            
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Navigation to coordinates failed: {e}")
            return False
        except subprocess.TimeoutExpired:
            rospy.logerr(f"Navigation to coordinates timed out")
            return False
   


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
            time.sleep(3)
            self.p.exec_action("gripperAction", "open")
            time.sleep(self.time_open_gripper)
            self.p.exec_action("gripperAction", "close")
            self.p.exec_action('speak', 'thank_you')

    
    def start(self):
        """Modified start method using interrupt-based trash collection"""
        self.p.exec_action('moveHead', '0.0_0.0')
        self.p.exec_action('moveTorso', '0.15')

        # Use interrupt-based trash detection
        self.phase_look_for_trash_with_interrupts()
         
        self.p.exec_action('moveHead', '0.0_0.0')
        self.p.exec_action('moveTorso', '0.15')

        self.phase_look_for_incorrectly_placed_objects()
         
        self.p.exec_action('moveHead', '0.0_0.0')
        self.p.exec_action('moveTorso', '0.25')

        # Second pass for any missed trash
        self.phase_look_for_trash_with_interrupts()
         
        self.p.exec_action('moveHead', '0.0_0.0')
        self.p.exec_action('moveTorso', '0.0_0.0')

        self.phase_look_for_people()

if __name__ == "__main__":
    p = PNPCmd()
    p.begin()
    gpsr = EGPSR(p)
    gpsr.start()
    # gpsr.obtain_quest_from_person()
    # gpsr.find_waving_people()
    p.end()
