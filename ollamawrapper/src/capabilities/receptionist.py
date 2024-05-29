from std_msgs.msg import String, Bool
import rospy
import capabilities.contexts as contexts


class ReceptionistPublisher:
    def __init__(self):
        self.ollama_name_pub = rospy.Publisher(
            "/ollama_name",
            String,
            queue_size=1,
            # latch=True,
        )
        self.ollama_drink_pub = rospy.Publisher(
            "/ollama_drink",
            String,
            queue_size=1,
            # latch=True,
        )

    def publish_name(self, person_name):
        self.ollama_name_pub.publish(person_name)

    def publish_drink(self, favourite_drink):
        self.ollama_drink_pub.publish(favourite_drink)

@contexts.context(["guest_name"])
def get_person_name(person_name):
    """Fetches the person's name from a statement such as 'My name is Bob', which would return "Bob",
    or 'Hello my name is Alice.' would return "Alice".
    The possible names are 'Alice', 'Bob', 'Charlie', and 'Dave'.

    Args:
        person_name (str): The person's name, parsed from the string
    """
    ReceptionistPublisher().publish_name(person_name)
    print("I think the person's name is %s" % person_name)

# @contexts.context(contexts.ALL)
@contexts.context(["guest_drink"])
def get_favourite_drink(favourite_drink):
    """Parses the person's favorite drink from a statement. For example, 'My favourite drink is Pepsi' would return "Pepsi",
    and 'My favourite drink is Coke' would return "Coke".

    Args:
        favorite_drink (str): The person's favorite drink, parsed from the string
    """
    ReceptionistPublisher().publish_drink(favourite_drink)
    print("I think the person's favourite drink is %s" % favourite_drink)

@contexts.context(["guest_drink"])
def get_favorite_drink(favorite_drink):
    get_favourite_drink(favorite_drink)


def t_person_name(person_name):
    get_person_name(person_name)

def person_name(person_name):
    get_person_name(person_name)

def t_favorite_drink(favourite_drink):
    get_favorite_drink(favourite_drink)

