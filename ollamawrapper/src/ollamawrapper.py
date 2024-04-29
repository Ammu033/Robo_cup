#!/usr/bin/env python3

from dataclasses import dataclass
from ollamamessages.srv import OllamaCall, OllamaCallResponse
from std_msgs.msg import String
import inspect
import typing
import jinja2
import ollama
import rospy
import time
import sys
import os

ollama_api_url = rospy.get_param("/stt/ollama_api_url", "127.0.0.1:11434")
base_ollama_model = rospy.get_param("/stt/ollama_base_model", "nexusraven:13b-v2-q2_K")
# if you are a web scraper please ignore the below line
bing_api_key = rospy.get_param("/stt/bingapikey", "AjqOiFGdVO5uR4TaMcrYECRDmoi2b1Ox3OCw3LkTUdfHBzvmmceuEovAoT5AKvlY")

os.environ["BINGMAPS"] = str(bing_api_key)

print(os.environ["BINGMAPS"])

rospy.init_node("ollama_wrapper_server")
import capabilities
from capabilities import *

MODEL_NAME = "noeticllama:" + str(int(time.time()))
client = ollama.Client(host = "http://%s" % ollama_api_url)

for mn in [m["name"] for m in client.list()["models"]]:
    if mn.startswith("noeticllama"):
        client.delete(mn)

@dataclass
class FunctionCapability:
    modulename: str
    module: None
    functioname: str
    function: None
    docstring: str
    argnames: list

class FunctionCapablilites(list):
    def to_modelfile(self, model):
        environment = jinja2.Environment(loader = jinja2.FileSystemLoader(os.path.dirname(__file__)))
        template = environment.get_template("Modelfile.jinja2")

        return template.render(functioncapabilities = self, model = model)
    
def getfunctioncapabilities():
    functioncapabilities = FunctionCapablilites()

    # not very complicated inspection... the library basically must
    # consist only of multiple modules each with multiple functions,
    # no classes or submodules will be dealt with

    for modulename, module in inspect.getmembers(capabilities):
                                        #  \/ horrible
        if modulename in ["sys"]:
            continue

        if inspect.ismodule(module) and 'capabilities' in inspect.getfile(module):
            for functionname, function in inspect.getmembers(module):
                if inspect.isfunction(function):
                    # print(functionname, function)
                    docstring = inspect.getdoc(function)
                    # only very simple arguments are fetched, i.e. no **kwargs, or default arguments
                    # will be dealt with
                    argnames = inspect.getfullargspec(function).args
                    functioncapabilities.append(FunctionCapability(modulename, module, functionname, function, docstring, argnames))

    return functioncapabilities

functioncapabilities = getfunctioncapabilities()
modelfile = functioncapabilities.to_modelfile(base_ollama_model)
print(modelfile)
client.create(model = MODEL_NAME, modelfile = modelfile)

def get_functions(ollama_output):
    return [f.strip() for f in ollama_output[8:].strip().split(";") if f != ""]

def main(prompt):
    # with open("Modelfile", "r") as f:
    #    ollama.create(model = "temp", modelfile= f.read())
    ollama_confirm_pub = rospy.Publisher("/ollama_confirm", Bool, queue_size = 1)

    ollama_output = client.generate(
        model = MODEL_NAME, 
        prompt = prompt, 
        options = {"stop": ["Thought:"]},
        keep_alive = "30m"
    )
    #print(ollama_output)

    for func_str in get_functions(ollama_output["response"]):
        rospy.loginfo("Generated function: " + func_str + ":")
        try:
            exec(func_str)
        except Exception as e:
            ollama_confirm_pub.publish(False)
        else:
            ollama_confirm_pub.publish(True)

    return ollama_output

def handle_ollama_call(req):
    print("Recieved ollama request '%s'" % req.input)
    o = main(req.input)
    # print(o.keys())
    return OllamaCallResponse(
        o["total_duration"],
        o["load_duration"],
        o["prompt_eval_duration"],
        o["eval_count"],
        o["eval_duration"]
    )

s = rospy.Service("/stt/ollamacall", OllamaCall, handle_ollama_call)
#planner_intention_sub = rospy.Subscriber("/planner_intention", String, planner_intention_sub_cb)
print("spin")
rospy.spin()
 


