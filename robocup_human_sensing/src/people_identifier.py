#! /usr/bin/env python3 

import rospy
from robocup_human_sensing.msg import RegionOfInterest2D
from robocup_human_sensing.msg import SoftBiometricsList
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge
from std_msgs.msg import String , ColorRGBA
from face_id import face_identifier
from age_id import age_identifier
from gender_id import gender_identifier
#from color_detector import color_id
import mediapipe as mp
import math
import random
import numpy as np
import webcolors

#camera_topic_rgb=rospy.get_param("/hs_people_identification/camera_topic_rgb")
camera_topic_rgb = '/xtion/rgb/image_raw'
#camera_topic_depth=rospy.get_param("/hs_people_identification/camera_topic_depth") 
#database_direct=rospy.get_param("/hs_people_identification/database_direct")
database_direct = "/home/lcastor/ros_ws/src/LCASTOR/robocup_human_sensing/database/"
#models_direct=rospy.get_param("/hs_people_identification/models_direct")
models_direct = "/home/lcastor/ros_ws/src/LCASTOR/robocup_human_sensing/weights/"

mp_face_detection = mp.solutions.face_detection

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


class Person:
    last_id = 0
    def __init__(self):
        self.id = "".join(random.choices("123456789010", k=3))
        self.nb_frames_visible = 1
        self.bb = None
        self.emp_name = None
        self.emp_dress_color = None
        self.emp_hair_color = None
        self.emp_skin_color = None
        self.ready = False

    
    def initialise_publisher(self):
        if self.ready:
            return
        self.roi_pub = rospy.Publisher("/humans/person/%s/roi" % self.id, RegionOfInterest , queue_size = 1)
        self.cropped_pub = rospy.Publisher("/humans/person/%s/cropped" % self.id , Image , queue_size =1)
        self.name_pub = rospy.Publisher("/humans/people/%s/name" % self.id, String , queue_size= 1)
        #self.dress_color = rospy.Publisher("/humans/people/%s/dress_color"  % self.id , ColorRGBA , queue_size = 1)
        #self.hair_color = rospy.Publisher("/humans/people/%s/hair_color"  %self.id , ColorRGBA , queue_size =1)
        rospy.loginfo("New Person : %s" % self)
        #self.skin_color = rospy.Publisher("/humans/people/%s/skin_color" %self.id , ColorRGBA , queue_size= 1)
        self.ready = True
    
    def publish(self):
        if not self.ready:
            rospy.logerr("Trying to publish but publishers have not been created")
            return
        self.roi_pub.publish(self.bb)
        self.name_pub.publish(self.emp_name)
        #self.dress_color.publish(self.emp_dress_color)
        #self.hair_color.publish(self.emp_hair_color)
        #self.skin_color.publish(self.emp_skin_color)

    def publish_images (self, src_image):
        if not self.ready :
            rospy.logerr("Trying to publish but publishers have not been created yet")
            return
        
        self.cropped_pub.publish(CvBridge().cv2_to_imgmsg(src_image[self.bb.y_offset : self.bb.y_offset + self.bb.height , 
                                                                    self.bb.x_offset : self.bb.x_offset + self.bb.width] , encoding = "bgr8"))
    
    def delete(self):

        if not self.ready:
            return

        rospy.loginfo(
            "Face [%s] lost. It remained visible for %s frames"
            % (self, self.nb_frames_visible)
        )

        self.roi_pub.unregister()
        self.cropped_pub.unregister()
        self.name_pub.unregister()
        #self.dress_color.unregister()
        #self.hair_color.unregister()
        #self.skin_color.unregister()

        self.ready = False

class RosPeopleDetect :
    def __init__(self):
        self.detector = mp_face_detection.FaceDetection(model_selection = 1 , min_detection_confidence = 0.8)
        self.name = face_identifier(database_direct,models_direct)
        #self.gender = gender_identifier(models_direct)
        #self.age = age_identifier(models_direct)
        #self.dress_color_ = color_id(models_direct)
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
    
    def closest_color(self, requested_colour):
        min_colours = {}
        for key, name in webcolors.CSS3_HEX_TO_NAMES.items():
            r_c, g_c, b_c = webcolors.hex_to_rgb(key)
            rd = (r_c - requested_colour[0]) ** 2
            gd = (g_c - requested_colour[1]) ** 2
            bd = (b_c - requested_colour[2]) ** 2
            min_colours[(rd + gd + bd)] = name
        return min_colours[min(min_colours.keys())]
    def get_color_name(self, requested_colour):
        try:
            closest_name = actual_name = webcolors.rgb_to_name(requested_colour)
        except ValueError:
            closest_name = self.closest_color(requested_colour)
            actual_name = None
        return actual_name, closest_name
    
    def callback(self, image_data):

        image = CvBridge().imgmsg_to_cv2(image_data , desired_encoding = "bgr8")
        img_height , img_width , _ = image.shape
        rospy.loginfo('waiting for people_roi')
        roi_data = rospy.wait_for_message('/people_roi' , RegionOfInterest2D)
        biometrics = SoftBiometricsList()
        currentIds = []
        knownIds = list(self.knownPeople.keys())
        ids_a = []
        emp_dress_list = []
        for i in range(len(roi_data.ids)):
            ids_a.append(roi_data.ids[i])
            emp_dress_list.append(roi_data.ids[i])
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
                person.nb_frames_visible += 1
                if person.nb_frames_visible == 2:
                    person.initialise_publisher()
                person.bb = bb
                name_id = person.emp_name
                emp_dress = person.emp_dress_color

            else :
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
                if learn == 1:
                    name_id = int(self.name.register_new_face(cropped_img , face_x , face_y , face_w , face_h))
                    is_found = True
                else:
                    name_id  , is_found  = self.name.name_teller(cropped_img , face_x , face_y , face_w , face_h)
                #gender_id = self.gender.gender_teller(cropped_img , face_x , face_y , face_w , face_h)
                #age_id = self.age.age_teller(cropped_img , face_x , face_y , face_w , face_h)
                #biometrics.age.append(age_id)
                #biometrics.gender.append(gender_id)
                if not is_found:
                    continue
                name_msg.data = name_id
                person.emp_name = name_id
                #pr_mask = self.dress_color_.Color(image)
                #for j in range(3):
                #    image[: , : , j] = np.multiply(image[: , :  , j] , pr_mask[: , : , 1])
                #print(image.shape)
                #cropped_dress = image[roi_data.y[i] : roi_data.y[i] + roi_data.h[i],
                #                    roi_data.x[i] : roi_data.x[i] + roi_data.w[i]]
                #dress_color = np.mean(cropped_dress[cropped_dress[ : , : , 1] != 0 ] , axis =0)
                #print(dress_color)
                #for j in range(3):
                #    image[: , : , j] = np.multiply(image[: , :  , j] , pr_mask[: , : , 1])
                #print(image.shape)
                #cropped_dress = image[roi_data.y[i] : roi_data.y[i] + roi_data.h[i],
                #                    roi_data.x[i] : roi_data.x[i] + roi_data.w[i]]
                #dress_color = np.mean(cropped_dress[cropped_dress[ : , : , 1] != 0 ] , axis =0)
                #dc = self.get_color_name([dress_color[2] , dress_color[1] , dress_color[0]])[1]
                #person.emp_dress_color  = dc
                #emp_dress = dc
                #file = open(str(name_id)+'.txt' , 'w')
                #file.write("%s \n %s \n %s \n" % (str(dc) , str(0) , str(0) ))
                #file.close()
                #person.dress_color = dress_filter[roi_data.y[i] : roi_data.y[i] + roi_data.h[i],
                #                    roi_data.x[i] : roi_data.x[i] + roi_data.w[i]]
                #print(person.dress_color.shape)
                #name_id = name_msg.data
                # Color based Info to be added 
                #skin_color
                #hair_color
                #dress_color
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
        
        for _, people in self.knownPeople.items():
            if people.ready:
                if not self.is_shutting_down:
                    person.publish()
                if not self.is_shutting_down:
                    person.publish_images(image)

if __name__=="__main__":
    rospy.init_node('people_identifier')
    detector = RosPeopleDetect()
    rospy.spin()
