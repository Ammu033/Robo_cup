#!/usr/bin/env python3

from dataclasses import dataclass
from ollamamessages.srv import OllamaCall, OllamaCallResponse
from ollamamessages.msg import OllamaResponse
from std_msgs.msg import String
import subprocess
import platform
import inspect
import tempfile
import typing
import jinja2
import ollama
import rospy
import time
import sys
import ast
import os
import re

sys.path.insert(1, os.path.join(os.path.dirname(__file__), "..", "..", "lcastor_actions"))
import gotoRoom

# ollama_api_url = rospy.get_param("/gpsr/ollama_api_url", "192.168.69.253:11434")
ollama_decomposition_model = rospy.get_param("/gpsr/ollama_decomposition_model", 'llama3')
# ollama_decomposition_model = rospy.get_param("/gpsr/ollama_decomposition_model", "deepseek-coder:6.7b")

import capabilities

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import time
import pnp_cmd_ros
from pnp_cmd_ros import *

class GPSRNode:
    def __init__(self):
        s = rospy.Service("/gpsr/task_decomposition", OllamaCall, self.handle_decomposition_call)
        self.intent_publisher = rospy.Publisher("/planner_intention", String, queue_size = 1)
        print("Node started correctly")
        rospy.spin()

    def parse_decorators(self, source):
        # yes these two functions are almost identical to the ones in ollamawrapper
        # but importing ROS stuff as python libraries causes wierd stuff to happen
        # alas i do not have enough experience with ROS programming
        for line in source.split("\n"):
            if line.startswith("@"):
                f = ast.parse(line[1:]).body[0].value 
                if f.func.value.id == "contexts" and f.func.attr == "context":
                    first_arg = f.args[0]
                    if type(first_arg) is ast.Attribute:
                        if first_arg.attr == "ALL" and first_arg.value.id == "contexts":
                            return capabilities.contexts.ALL
                    else:
                        # it's a list
                        o = []
                        for i in first_arg.elts:
                            o.append(i.value)
                        return o

    def gpsr_commands(self):
        sources = []
        for modulename, module in inspect.getmembers(capabilities):
                                        #  \/ horrible
            if modulename in ["sys", "time"]:
                continue
            if inspect.ismodule(module) and "gpsr" in inspect.getfile(module):
                for functionname, function in inspect.getmembers(module):
                    if inspect.isfunction(function):
                        source = inspect.getsource(function)
                        decorators = self.parse_decorators(source)
                        if decorators is None: continue
                        # print(decorators)
                        if "gpsr" in decorators:
                            sources.append(source)
        return sources
    
    def parse_ollama_call(self, ollama_text):
        if "```" not in ollama_text:
            return ollama_text
        # print(ollama_text.split("```"))
        s = ollama_text.split("```")[1]
        if s.startswith("python"):
            s = s[6:]
        return s

    def handle_decomposition_call(self, req):
        human_str = req.input
        rospy.loginfo("Recieved task decomposition string: '%s'" % human_str)

        func_name = "do_func"
        # locations_in_scene = list(gotoRoom.ROOM_DICT.keys())
        if rospy.get_param("/arena") == "arena_b":
            locations_in_scene = list(gotoRoom.ROOM_DICT_B.keys())
        else:
            locations_in_scene = list(gotoRoom.ROOM_DICT_C.keys())

        objects_in_scene = ["toy", "sugar", "food"]
        rospy.loginfo("Locations in scene: %s" % str(locations_in_scene))

        # print(self.gpsr_commands())
        source_functions = "\n".join(self.gpsr_commands())

        prompt = \
        f"You are the AI controlling a robot. You have access to the following functions: ```{source_functions}```.\
        Given a locations list containing the locations in the scene as a list of string location names, \
        and a list of objects in the scene as a list of string object names, \
        You get a user request the robot to do: '{human_str}'. Create a function for completing the task. \
        Always set the `p` parameter to the variable `p`, and assume it is already defined. \
        `p` does not have any methods. You should never call any methods of `p`. \
        The function signature must be: {func_name}(p, locations: list)->str. You cannot write anything outside the function. \
        The function should not have any decorators. \
        You do not need to def the functions in the context. \
        You do not need to explain your code. \
        We have the following locations in the scene: {locations_in_scene}. \
        You may only use the functions provided to you in the context. Do not use print(str) but use engine_say(str) instead. \
        The engine_say(str) also speaks out loud. You need to call the functions I gave you to complete the task. \
        You should start by saying a repetition of the task: for example 'my task is to...'. \
        You may import and use modules in the standard python library. \
        You do not need to check of an object or location is in the scene. \
        Today is friday. Your team's name is LCASTOR. Our team is based in the city of Lincoln. \
        For example, the task 'tell me what is the heaviest object on the sink' could call: \
        goto_location(p, location_name='sink') then identify_objects(p, what_to_idenfify='the heaviest object') \
        then go_back_to_me(p) then report_information(p). Another example is that 'lead Robin from the dinner table to the hallway' \
        could call `goto_location(p, location_name='sink')` then `ask_for_person(p, person_name='Robin'), then` \
        `guide_person(p)` and then finally `goto_location(p, location_name='hallway')` \
        After you finish your task you should always come back to me."

        ollama_api_url = rospy.get_param("/gpsr/ollama_api_url", "127.0.0.1:11434")
        rospy.loginfo("Using ollama HTTP API at %s" % ollama_api_url)
        client = ollama.Client(host = "http://%s" % ollama_api_url)
        # print(client.list())
        ollama_output = client.generate(
            model = ollama_decomposition_model, 
            prompt = prompt, 
            keep_alive = "0m",
            options = {"stop": ["```\n\n"]}
        )
        rospy.loginfo("Raw ollama response: =====\n%s\n=====" % ollama_output["response"])

        parsed_func = self.parse_ollama_call(ollama_output["response"])

        # print("To exec: ===\n%s\n===" % parsed_func)

        environment = jinja2.Environment(loader = jinja2.FileSystemLoader(os.path.dirname(__file__)))
        template = environment.get_template("gpsr_runner.py.jinja2")

        with tempfile.NamedTemporaryFile(suffix = ".py", mode = "w", delete = False) as f:
            f.write(template.render(
                cwd = os.path.dirname(__file__), 
                main_func = parsed_func, 
                func_call = "%s(p = p, locations = %s)" % (func_name, str(locations_in_scene))
            ))
            filename = f.name
        rospy.loginfo(str(filename))

        with open(filename, "r") as f:
            rospy.loginfo("******* %s consists of the following: *******\n\n%s\n****************************************" % (filename, f.read()))
        # subprocess.run(["tmux", "new-session", "-s", "gpsr_runner", "-d", "'", "python3", filename, "'"])

        # try:
        #     proc = subprocess.Popen(["python3", filename], stdout = subprocess.PIPE, stderr = subprocess.PIPE)
        #     while True:
        #         line = proc.stdout.readline()
        #         if not line:
        #             break
        #         rospy.loginfo("[GPSR Runner] %s" % line.rstrip().decode())
        #     while True:
        #         line = proc.stderr.readline()
        #         if not line:
        #             break
        #         rospy.logwarn("[GPSR Runner] %s" % line.rstrip().decode())
        # except KeyboardInterrupt:
        #     proc.terminate()
        #     rospy.loginfo("Proc terminated")

        return OllamaCallResponse(
            ollama_output["total_duration"],
            ollama_output["load_duration"],
            ollama_output["prompt_eval_duration"],
            ollama_output["eval_count"],
            ollama_output["eval_duration"]
        )


if __name__ == "__main__":
    rospy.init_node("gpsr_node")
    gpsr = GPSRNode()