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

ollama_api_url = rospy.get_param("/stt/ollama_api_url", "127.0.0.1:11434")
ollama_decomposition_model = rospy.get_param("/stt/ollama_decomposition_model", "llama3")

rospy.init_node("gpsr_node")

class GPSRNode:
    def __init__(self):
        print("Node started correctly")
        s = rospy.Service("/gpsr/task_decomposition", OllamaCall, self.handle_decomposition_call)
        self.intent_publisher = rospy.Publisher("/planner_intention", String, queue_size = 1)
        rospy.spin()

    def get_modelfile(self):
        environment = jinja2.Environment(loader = jinja2.FileSystemLoader(os.path.dirname(__file__)))
        template = environment.get_template("Modelfile_gpsr.jinja2")

        return template.render(ollama_decomposition_model = ollama_decomposition_model)
    
    def create_model(self):
        model_name = "taskdecomposer:" + str(int(time.time()))
        client = ollama.Client(host = "http://%s" % ollama_api_url)

        for mn in [m["name"] for m in client.list()["models"]]:
            if mn.startswith("taskdecomposer"):
                client.delete(mn)

        modelfile = self.get_modelfile()
        # print(modelfile)
        client.create(model = model_name, modelfile = modelfile)
        return client, model_name
    
    def ollama_call_parser(self, raw_ollama):
        return [i.strip() for i in re.split(r"\d.", raw_ollama) if i != ""]
    
    def do_task(self, subtask):
        try:
            service_call = rospy.ServiceProxy("/stt/ollamacall", OllamaCall)
            response = service_call(input = subtask)
            print(response)
        except Exception as e:
            print("Ollama failed: ", str(e))
        else:
            generated_cmd = rospy.wait_for_message(
                "/ollama_output", String, timeout = 2
            )
            rospy.loginfo("Response: %s" % generated_cmd.data)

    def handle_decomposition_call(self, req):
        human_str = req.input
        rospy.loginfo("Recieved task decomposition string: '%s'" % human_str)
        client, model_name = self.create_model()

        ollama_output = client.generate(
            model = model_name, 
            prompt = human_str, 
            keep_alive = "0m"
        )
        # rospy.loginfo("Raw ollama response: '%s'" % ollama_output["response"])
        tasks = self.ollama_call_parser(ollama_output["response"])
        rospy.loginfo("Parsed ollama call: " + str(tasks))

        self.intent_publisher.publish("gpsr")
        for task in tasks:
            self.do_task(task)

        return OllamaCallResponse(
            ollama_output["total_duration"],
            ollama_output["load_duration"],
            ollama_output["prompt_eval_duration"],
            ollama_output["eval_count"],
            ollama_output["eval_duration"]
        )


if __name__ == "__main__":
    gpsr = GPSRNode()