import rospy
import capabilities.contexts as contexts
import capabilities.receptionist as receptionist

def publish_what_im_doing(what_im_doing):
    print(what_im_doing)
    receptionist.ReceptionistPublisher().publish_output('Generated subtask, I am going to "%s"' % what_im_doing)

@contexts.context(["gpsr"])
def goto_location(location_name):
    """Go to a named location name, such as 'side tables', 'bedroom' or 'cabinet' or 'bedside table'

    Args:
        location_name (str): The location name to go to, as a string
    """
    publish_what_im_doing("goto_location(location_name='%s')" % location_name)

@contexts.context(["gpsr"])
def go_back_to_you():
    """Go back to you"""
    publish_what_im_doing("go_back_to_you()")

@contexts.context(["gpsr"])
def grasp_object(object_name):
    """Grasps a given object, for example 'fruit' or 'bowl'

    Args:
        object_name (str): The name of the object to grasp
    """
    publish_what_im_doing("grasp_object(object_name='%s')" % object_name)

@contexts.context(["gpsr"])
def ask_for_person(person_name):
    """Ask for a person with a given name, for example 'Angel' or 'Morgan'

    Args:
        person_name (str): The person's name to ask for
    """
    publish_what_im_doing("ask_for_person(person_name='%s')" % person_name)

@contexts.context(["gpsr"])
def identify_people(what_to_identify):
    """Identifies and counts the number of people in a room doing a given action.
    For example what_to_identify could be 'standing persons' or 'pointing to the right'

    Args:
        what_to_identify (str): Action to identify
    """
    publish_what_im_doing("identify_people(what_to_idenfify='%s')" % what_to_identify)

@contexts.context(["gpsr"])
def identify_object(what_to_idenfify):
    """Identify a given item, for example 'the biggest food item', or 'how many cleaning supplies'.
    Should also include what to identify, such as 'the largest item', 'the smallest item' or counting how many
    items there are.

    Args:
        what_to_idenfify (str): The biggest food item to identify
    """
    publish_what_im_doing("identify_object(what_to_idenfify='%s')" % what_to_idenfify)

@contexts.context(["gpsr"])
def report_information():
    """Report what you have just identified
    """
    publish_what_im_doing("report_information()")

@contexts.context(["gpsr"])
def salute():
    """Salute a person"""
    publish_what_im_doing("Salute")

@contexts.context(["gpsr"])
def done():
    publish_what_im_doing("done()")