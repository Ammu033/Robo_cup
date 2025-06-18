from std_msgs.msg import String, Bool
import rospy
import capabilities.contexts as contexts


class ReceptionistPublisher:
    def __init__(self):
        self.ollama_out_pub = rospy.Publisher(
            "/ollama_output",
            String,
            queue_size=1,
            latch=True,
        )

    def publish_output(self, output):
        self.ollama_out_pub.publish(output)


@contexts.context(["guest_name"])
def get_room_name(person_name):
    """Fetches the room name from a statement such as 'save this pose as kitchen', which would return "kitchen",
    or 'save this as room_one.' would return "room_one".
    The possible names are 'kitchen', 'room_1', 'groccery_room', and 'room_3'.

    Args:
        room_name (str): The room name, parsed from the string
    """
    ReceptionistPublisher().publish_output(person_name)
    print("I think the room name is %s" % person_name)

# @contexts.context(contexts.ALL)
@contexts.context(["guest_drink"])
def get_favourite_drink(drink):
    """Parses the person's favorite drink from a statement. For example, 'My favourite drink is Pepsi' would return "Pepsi",
    and 'My favourite drink is Coke' would return "Coke".

    Args:
        drink (str): The person's favorite drink, parsed from the string
    """
    ReceptionistPublisher().publish_output(drink)
    print("I think the person's favourite drink is %s" % drink)

@contexts.context(["guest_drink"])
def get_favorite_drink(drink):
    get_favourite_drink(drink)

@contexts.context(["affirm_deny"])
def parse_yes_no(affirm):
    """Parses an input string to get the intent from a yes or no question. Takes a boolean, which should be True if the input string
    is "Yes", or is otherwise positive intent, and should be False if the input string is "No" or otherwise negative intent.

    Args:
        affirm (bool): Boolean indicating a yes or no response, with True indicating yes, and False indicating No
    """
    if affirm:
        print("I think this was a positive response (YES)")
        ReceptionistPublisher().publish_output("yes")
    else:
        print("I think this was a negative response (NO)")
        ReceptionistPublisher().publish_output("no")

def rse_yes_no(affirm):
    parse_yes_no(affirm)

def t_room_name(room_name):
    get_room_name(room_name)

def room_name(room_name):
    get_room_name(room_name)

def t_favorite_drink(drink):
    get_favorite_drink(drink)

def t_favourite_drink(drink):
    get_favourite_drink(drink)
