import os
import sys
import tf
import math

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
import robocup_human_sensing
from sensing_classes import Person
from sensing_classes import face_identifier
import rospy
from robocup_human_sensing.msg import RegionOfInterest2D
from robocup_human_sensing.msg import SoftBiometricsList
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge
from std_msgs.msg import String , ColorRGBA
#from face_id import face_identifier
#from age_id import age_identifier
#from gender_id import gender_identifier
#from color_detector import color_id
import mediapipe as mp
import math
import random
import numpy as np
import webcolors

#from people_msgs.msg import People
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D
from AbstractAction import AbstractAction

camera_topic_rgb = '/xtion/rgb/image_raw'
#camera_topic_depth=rospy.get_param("/hs_people_identification/camera_topic_depth") 
#database_direct=rospy.get_param("/hs_people_identification/database_direct")
database_direct = "/home/lcastor/ros_ws/src/LCASTOR/robocup_human_sensing/database/"
#models_direct=rospy.get_param("/hs_people_identification/models_direct")
models_direct = "/home/lcastor/ros_ws/src/LCASTOR/robocup_human_sensing/weights/"
mp_face_detection = mp.solutions.face_detection
rospy.set_param('learn' , 0)
def normalized_to_pixel_coordinates(
    normalized_x: float,
    normalized_y: float,
    image_width: int,
    image_height: int,
):

    x_px = min(math.floor(normalized_x * image_width), image_width - 1)
    y_px = min(math.floor(normalized_y * image_height), image_height - 1)
    return x_px, y_px


MAX_ROIS_DISTANCE = 200

# max scale factor between two successive
# regions of interest to consider they
# belong to the same person
MAX_SCALING_ROIS = 2

# default size in pixels for the re-published faces
# can be changed via the ROS parameters
# /humans/faces/width and /humans/faces/height
cropped_face_width = 128
cropped_face_height = 128

class peopleDetection(AbstractAction):
    def _start_action(self):
        self.detector = mp_face_detection.FaceDetection(model_selection = 1 , min_detection_confidence = 0.8)
        self.name = face_identifier(database_direct,models_direct)
        #self.gender = gender_identifier(models_direct)
        #self.age = age_identifier(models_direct)
        #self.dress_color_ = color_id(models_direct)
        rospy.logerr(self.name)
        rospy.set_param('modelLoaded' , True)


        self.single_pub = rospy.Publisher("/h_boxes_tracked" , RegionOfInterest2D , queue_size =1)
        self.bio_pub = rospy.Publisher("/biodata" , SoftBiometricsList , queue_size= 1)
        self.knownPeople = {}
        
        self.is_shutting_down = False
        self.box_sub = rospy.Subscriber(camera_topic_rgb, Image, self.callback)
        rospy.loginfo('waiting for topics')

    
    def distance_rois(self, bb1, bb2):
        x1, y1 = bb1.x_offset + bb1.width / 2, bb1.y_offset + bb1.height / 2
        x2, y2 = bb2.x_offset + bb2.width / 2, bb2.y_offset + bb2.height / 2

        return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)

    def find_previous_match(self, bb):
        for _, person in self.knownPeople.items():
            prev_bb = person.bb
            if (
                self.distance_rois(prev_bb, bb) < MAX_ROIS_DISTANCE * MAX_ROIS_DISTANCE
                and 1 / MAX_SCALING_ROIS < prev_bb.width / bb.width < MAX_SCALING_ROIS
                and 1 / MAX_SCALING_ROIS < prev_bb.height / bb.height < MAX_SCALING_ROIS
            ):
                return person
        return None

    def SkipBoundingBox(self , rd):
        #area = rd[2] * rd[3]
        #rd[4] is img_width rd[5] is img_height
        condition_1 = rd[2] < (rd[4]*0.2)
        condition_2 = rd[3] < (rd[5]*0.8)
        condition_3 = rd[0] < (rd[4]*0.2) or rd[0] > (rd[4] * 0.8)
        return condition_1 or condition_2 or condition_3

    def callback(self, image_data):
        image = CvBridge().imgmsg_to_cv2(image_data , desired_encoding = "bgr8")
        img_height , img_width , _ = image.shape
        rospy.loginfo('waiting for people_roi')
        roi_data = rospy.wait_for_message('/people_roi' , RegionOfInterest2D)
        biometrics = SoftBiometricsList()
        currentIds = []
        knownIds = list(self.knownPeople.keys())
        ids_a = []
        #emp_dress_list = []
        for i in range(len(roi_data.ids)):
            print(roi_data.x[i] , roi_data.y[i] , roi_data.w[i] , roi_data.h[i] , img_width , img_height)
            ids_a.append(roi_data.ids[i])
            #emp_dress_list.append(roi_data.ids[i])
            bb = RegionOfInterest(
                max(0 , roi_data.x[i]),
                max(0 , roi_data.y[i]),
                min(img_height - roi_data.y[i] ,roi_data.h[i] ),
                min(img_width - roi_data.x[i] , roi_data.w[i]),
                True,
            )
            person = self.find_previous_match(bb)
            if person:
                currentIds.append(person.id)
                #person.nb_frames_visible += 1
                #if person.nb_frames_visible == 2:
                #    person.initialise_publisher()
                person.bb = bb
                name_id = person.emp_name
                emp_dress = person.emp_dress_color

            else :
                rospy.loginfo('Inside People Id and Recog')
                if (self.SkipBoundingBox([roi_data.x[i] , roi_data.y[i] , roi_data.w[i] , roi_data.h[i] , img_width , img_height])):
                    rospy.loginfo('Skipping Box')
                    continue
                cropped_img = image[roi_data.y[i] : roi_data.y[i] + roi_data.h[i],
                                    roi_data.x[i] : roi_data.x[i] + roi_data.w[i]]
                detection = self.detector.process(cropped_img ).detections
                if not detection:
                    continue
                r ,  c , _ = cropped_img.shape
                local_bb = detection[0].location_data.relative_bounding_box
                face_x , face_y = normalized_to_pixel_coordinates(local_bb.xmin , local_bb.ymin , c , r)
                face_w , face_h = normalized_to_pixel_coordinates(local_bb.width , local_bb.height , c , r)
                person = Person()
                person.bb = bb
                name_msg = String()
                learn = rospy.get_param('learn')
                is_found = False
                rospy.loginfo('Going into Id , Recog')
                #if self.SkipBoundingBox(roi_data.x[i] , roi_data.y[i] , roi_data.w[i] , roi_data.h[i])
                if learn == 1:

                    name_id = int(self.name.register_new_face(cropped_img , face_x , face_y , face_w , face_h))
                    rospy.set_param('LastSavedid' , name_id)
                    rospy.set_param('learn' , 0)
                    rospy.set_param('personSaved' , 1)
                    is_found = True
                else:
                    name_id  , is_found  = self.name.name_teller(cropped_img , face_x , face_y , face_w , face_h)
                if not is_found:
                    continue
                name_msg.data = name_id
                person.emp_name = name_id
                self.knownPeople[person.id] = person
            if name_id :
                ids_a[i] = (int(name_id))
                #emp_dress_list[i] = str(emp_dress)
        biometrics.ids = tuple(ids_a)
        #biometrics.dress_color  = tuple(emp_dress_list)
        roi_data.ids = tuple(ids_a)
        self.single_pub.publish(roi_data)
        self.bio_pub.publish(biometrics)
        
        for id in knownIds:
            if id not in currentIds:
                self.knownPeople[id].delete()
                del self.knownPeople[id]
    
    def _stop_action(self):
        self.box_sub.unregister()
        rospy.loginfo("Stopping PeopleDetection")
        #rospy.set_param(ROSPARAM, self.person_to_follow.name)
        self.params.append("done")
    
    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached

