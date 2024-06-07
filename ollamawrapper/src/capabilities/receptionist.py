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
def get_person_name(person_name):
    """Fetches the person's name from a statement such as 'My name is Bob', which would return "Bob",
    or 'Hello my name is Alice.' would return "Alice".
    The possible names are 'Alice', 'Bob', 'Charlie', and 'Dave'.

    Args:
        person_name (str): The person's name, parsed from the string
    """
    ReceptionistPublisher().publish_output(person_name)
    print("I think the person's name is %s" % person_name)

# @contexts.context(contexts.ALL)
@contexts.context(["guest_drink"])
def get_favourite_drink(favourite_drink):
    """Parses the person's favorite drink from a statement. For example, 'My favourite drink is Pepsi' would return "Pepsi",
    and 'My favourite drink is Coke' would return "Coke".

    Args:
        favorite_drink (str): The person's favorite drink, parsed from the string
    """
    ReceptionistPublisher().publish_output(favourite_drink)
    print("I think the person's favourite drink is %s" % favourite_drink)

@contexts.context(["guest_drink"])
def get_favorite_drink(favorite_drink):
    get_favourite_drink(favorite_drink)

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

def t_person_name(person_name):
    get_person_name(person_name)

def person_name(person_name):
    get_person_name(person_name)

def t_favorite_drink(favourite_drink):
    get_favorite_drink(favourite_drink)

