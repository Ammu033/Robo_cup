#!/usr/bin/env python3

from dataclasses import dataclass
from ollamamessages.srv import OllamaCall, OllamaCallResponse
from ollamamessages.msg import OllamaResponse
from std_msgs.msg import String
import platform
import inspect
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

ollama_api_url = rospy.get_param("/stt/ollama_api_url", "127.0.0.1:11434")
ollama_decomposition_model = rospy.get_param("/stt/ollama_decomposition_model", 'deepseek-coder-v2:latest')
# ollama_decomposition_model = rospy.get_param("/stt/ollama_decomposition_model", "deepseek-coder:6.7b")

import capabilities
from capabilities.gpsr import *

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
        return ollama_text.split("```python")[-1]

    def handle_decomposition_call(self, req):
        human_str = req.input
        rospy.loginfo("Recieved task decomposition string: '%s'" % human_str)

        func_name = "do_func"
        locations_in_scene = list(gotoRoom.ROOM_DICT.keys())
        objects_in_scene = ["toy", "sugar", "food"]
        rospy.loginfo("Locations in scene: %s" % str(locations_in_scene))

        # print(self.gpsr_commands())
        source_functions = "\n".join(self.gpsr_commands())

        prompt = \
        f"You are the AI controlling a robot. You have access to the following functions: ```{source_functions}```.\
        Given a locations list containing the locations in the scene as a list of string location names, \
        and a list of objects in the scene as a list of string object names, \
        You get a user request the robot to do: '{human_str}'. Create a function for completing the task \
        The function signature must be: {func_name}(locations: list, objects: list)->str. You cannot write anything outside the function. You do not need to def the functions in the context. \
        In addition, if it fails, you should print out why it fails. You do not need to return this. \
        You do not need to explain your code. Call the function {func_name}.  \
        We have the following locations in the scene: {locations_in_scene}. \
        There are the following objects in the scene: {objects_in_scene}. \
        You may only use the functions provided to you in the context. Do not use print(str) but use engine_say(str) instead.\
        You need to call the functions I gave you to complete the task. If a location or object is not in the scene, print out that it's not in the scene."

        client = ollama.Client(host = "http://%s" % ollama_api_url)
        ollama_output = client.generate(
            model = ollama_decomposition_model, 
            prompt = prompt, 
            keep_alive = "0m",
            options = {"stop": ["```\n"]}
        )
        rospy.loginfo("Raw ollama response: =====\n%s\n=====" % ollama_output["response"])

        parsed_func = self.parse_ollama_call(ollama_output["response"])
        parsed_func += "\n\n%s(locations = %s, objects = %s)" % (func_name, str(locations_in_scene), str(objects_in_scene))

        print("To exec: ===\n%s\n===" % parsed_func)

        try:
            exec(parsed_func)
        except Exception as e:
            rospy.loginfo(e)


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