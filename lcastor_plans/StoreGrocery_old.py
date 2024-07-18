import os
import sys
import time

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)
import pnp_cmd_ros
from pnp_cmd_ros import *
from pickUp_bag import pickUp_bag
from PersonFollowing import PersonFollowing
from OfferGripper import OfferGripper
import rospy
from vision_msgs.msg import Detection3DArray
from AskConfirmation import AskConfirmation

labels_lookup = ["chips_can",
"master_chef_can",
"cracker_box",
"sugar_box",
"tomato_soup_can",
"mustard_bottle",
"tuna_fish_can",
"pudding_box",
"gelatin_box",
"potted_meat_can",
"banana",
"strawberry",
"apple",
"lemon",
"peach",
"pear",
"orange",
"plum",
"bleach_cleanser",
"bowl",
"mug",
"plate",
"fork",
"spoon",
"knife",
"mini_soccer_ball",
"softball",
"baseball",
"tennis_ball",
"racquetball",
"golf_ball",
"dice",
"rubiks_cube"]

categories_lookup = [	1,
                        1,
                        1,
                        1,
                        1,
                        1,
                        1,
                        1,
                        1,
                        1,
                        2,
                        2,
                        2,
                        2,
                        2,
                        2,
                        2,
                        2,
                        3,
                        3,
                        3,
                        3,
                        3,
                        3,
                        3,
                        4,
                        4,
                        4,
                        4,
                        4,
                        4,
                        4,
                        4]

def StoreGrocery(p):  


    p.exec_action("speak", "Can_you_please_open_the_door_for_me_?")
    while(not p.get_condition("IsDoorOpen")): time.sleep(0.1)
    p.exec_action("speak", "Thank_you_for_opening_the_door")

    # p.exec_action("gotoRoom", "r_inspectionpoint")

    p.exec_action("speak", "I_will_now_help_you_store_groceries.")
    
    while True:
        # 1. Go to Table
        p.exec_action('gotoRoom', 'r_coffetable')
        p.exec_action("armAction", "home")
        p.exec_action("gripperAction", "close")
        
        # 2. Look at the table
        p.exec_action('moveHead', '-0.1_-0.56')
        p.exec_action('moveTorso', '0.2')

        # 3. Activate detector
        detections = rospy.wait_for_message("/xtion/rgb/detections_3d", Detection3DArray, timeout=10)
        #print(detections.detections.results)
        detections = [d.results[0].id for d in detections.detections]
        if len(detections) == 0:
            p.exec_action("speak", "I_cannot_store_anything_else._Please_proceed_on_your_own.")
            return
         
        obj_name = labels_lookup[detections[0] - 1] 
        category_obj = categories_lookup[detections[0] - 1]  

        p.exec_action("armAction", "offer")
        for i in range(2):
            p.exec_action("speak", "Can_you_please_put_the_{:s}_in_my_hand.".format(obj_name))
            p.exec_action("speak", "Please_say_yes_when_I_can_grab_it.")
            if AskConfirmation(p):
                break
        p.exec_action("gripperAction", "close")


        p.exec_action('gotoRoom', 'r_cabinet')
        p.exec_action('moveHead', '0.0_0.0')
        p.exec_action('moveTorso', '0.2')
        
        detections_shelf = rospy.wait_for_message("/xtion/rgb/detections_3d", Detection3DArray, timeout=10)
        #detections_shelf = [d.results[0].id for d in detections_shelf.detections]
        
        obj_placed = False
        for d in detections_shelf.detections:
            id = d.results[0].id
            if categories_lookup[id-1] == category_obj:
                shelf_obj_name = labels_lookup[id - 1] 
                p.exec_action("speak", "Can_you_please_place_the_{:s}_in_the_shelf_next_to_the_{:s}".format(obj_name, shelf_obj_name))
                #p.exec_action("armAction", "offer")
                for i in range(2):
                    p.exec_action("speak", "Please_say_yes_when_you_are_ready_to_take_it.")
                    if AskConfirmation(p):
                        p.exec_action("gripperAction", "open")
                        time.sleep(5)
                        break
                obj_placed = True
                break
        
        if not obj_placed:
            target_loc = [3, "left"]
            p.exec_action("speak", "Can_you_please_place_the_{:s}_in_shelf_{:d}_on_the_{:s}".format(obj_name, target_loc[0], target_loc[1]))   
            for i in range(2):
                p.exec_action("speak", "Please_say_yes_when_you_are_ready_to_take_it.")
                if AskConfirmation(p):
                    p.exec_action("gripperAction", "open")
                    time.sleep(5)
                    break

if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    StoreGrocery(p)

    p.end()

