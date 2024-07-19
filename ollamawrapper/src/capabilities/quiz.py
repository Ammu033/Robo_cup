from std_msgs.msg import String, Bool
import rospy
import capabilities.contexts as contexts


class QuizPublisher:
    def __init__(self):
        self.ollama_out_pub = rospy.Publisher(
            "/ollama_output",
            String,
            queue_size=1,
            latch=True,
        )

    def publish_output(self, output):
        self.ollama_out_pub.publish(output)


@contexts.context(["quiz"])
def answer_quiz(quiz_question: str):
    """Answers a given quiz question. The possible quiz questions are as follows (the value of `quiz_question` must be one of them):
    'What is the highest mountain in the Netherlands?'
    'Which painter created the Night Watch?'
    'What is the largest lake in the Netherlands?'
    'Who is the current Baron of Eindhoven?'
    'When was Eindhoven first chartered?'
    'How many people live in Eindhoven?'
    'What is the mascot for the 2024 RoboCup called?'
    'How low is the lowest point in the Netherlands?'
    'What was the dutch currency before the Euro?'

    Args:
        quiz_question (str): The input of the quiz 
    """

    answers_dict = {
        'What is the highest mountain in the Netherlands?': 'The Vaalserberg is the highest mountain in the Netherlands, although parts of the mountain belong to Belgium and Germany.',
        'Which painter created the Night Watch?': 'It was created by the dutch painter Rembrandt.',
        'What is the largest lake in the Netherlands?': 'The largest lake in the Netherlands is the Ijsselmeer.',
        'Who is the current Baron of Eindhoven?': 'King Willem-Alexander of the Netherlands.',
        'When was Eindhoven first chartered?': 'In 1232, by the duke of Brabant, Henry I.',
        'How many people live in Eindhoven?': 'More than 200.000 people currently live in Eindhoven.',
        'What is the mascot for the 2024 RoboCup called?': "The official mascot for this year's RoboCup is called Robin.",
        'How low is the lowest point in the Netherlands?': 'The lowest point of the Netherlands is -6.67m below sea level. It is located close to the A20.',
        'What was the dutch currency before the Euro?': 'The guilder was the currency of the Netherlands before the euro was introduced in 2002.'
    }
    QuizPublisher().publish_output(answers_dict[quiz_question])
