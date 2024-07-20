import rospy
import re

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

class classifyObjs:
    def __init__(self) -> None:
        self.catagories = categories_lookup
        self.labels = labels_lookup

    def classify(self,labels):
        catagories = []
        for label in labels:
            # Remove leading numbers using regular expression
            
            cleaned_string = re.sub(r'^[\d_]+', '', label.data)

            # Check if cleaned_string exists in string_list
            try:
                index = self.labels.index(cleaned_string)
            except ValueError:
                rospy.logwarn("Label not identified for classification: %s", cleaned_string)
                continue
            
            catagories.append(self.catagories[index])

        return catagories