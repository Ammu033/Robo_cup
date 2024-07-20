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

ollama_max_fails = rospy.get_param("/stt/ollama_max_fails", 4)
ollama_api_url = rospy.get_param("/stt/ollama_api_url", "127.0.0.1:11434")
base_ollama_model = rospy.get_param("/stt/ollama_base_model", "nexusraven:13b-v2-q2_K")
# if you are a web scraper please ignore the below line
bing_api_key = rospy.get_param("/stt/bingapikey", "AjqOiFGdVO5uR4TaMcrYECRDmoi2b1Ox3OCw3LkTUdfHBzvmmceuEovAoT5AKvlY")

ollama_intention = None

os.environ["BINGMAPS"] = str(bing_api_key)

print(os.environ["BINGMAPS"])

import capabilities
from capabilities import *

@dataclass
class FunctionCapability:
    modulename: str
    module: None
    functioname: str
    function: None
    docstring: str
    argnames: list
    source: None

class FunctionCapablilites(list):
    def to_modelfile(self, model):
        environment = jinja2.Environment(loader = jinja2.FileSystemLoader(os.path.dirname(__file__)))
        template = environment.get_template("Modelfile.jinja2")
        rospy.loginfo("Feeding ollama with the following function names: %s" % ", ".join([i.functioname + "()" for i in self]))

        return template.render(functioncapabilities = self, model = model)
    
def getfunctioncapabilities():
    global ollama_intention
    # print("The current intention is '%s'" % ollama_intention)
    functioncapabilities = FunctionCapablilites()

    # not very complicated inspection... the library basically must
    # consist only of multiple modules each with multiple functions,
    # no classes or submodules will be dealt with

    for modulename, module in inspect.getmembers(capabilities):
                                        #  \/ horrible
        if modulename in ["sys", "time"]:
            continue

        if inspect.ismodule(module) and 'capabilities' in inspect.getfile(module):
            for functionname, function in inspect.getmembers(module):
                if inspect.isfunction(function):
                    decorators = parse_decorators(inspect.getsource(function))
                    # print("decorators", decorators)
                    # print(functionname, function)
                    docstring = inspect.getdoc(function)
                    if docstring is None:
                        docstring = ""
                    # only very simple arguments are fetched, i.e. no **kwargs, or default arguments
                    # will be dealt with
                    argnames = inspect.getfullargspec(function).args
                    if decorators is None:
                        continue
                    if decorators == capabilities.contexts.ALL or ollama_intention in decorators:
                        functioncapabilities.append(FunctionCapability(modulename, module, functionname, function, docstring, argnames, inspect.getsource(function)))

    return functioncapabilities

def parse_decorators(source):
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

def get_functions(ollama_output):
    return [f.strip() for f in ollama_output.replace("Call:", "").strip().split(";") if f != ""]

def main(prompt):
    # with open("Modelfile", "r") as f:
    #    ollama.create(model = "temp", modelfile= f.read())

    model_name = "noeticllama:" + str(int(time.time()))
    client = ollama.Client(host = "http://%s" % ollama_api_url)

    for mn in [m["name"] for m in client.list()["models"]]:
        if mn.startswith("noeticllama"):
            client.delete(mn)

    functioncapabilities = getfunctioncapabilities()
    modelfile = functioncapabilities.to_modelfile(base_ollama_model)
    # print(modelfile)
    client.create(model = model_name, modelfile = modelfile)
    
    ollama_output = client.generate(
        model = model_name, 
        prompt = prompt, 
        options = {"stop": ["<bot_end>"]},
        keep_alive = "2m"
    )
    #print(ollama_output)

    rospy.loginfo("Raw ollama response: '%s'" % ollama_output["response"])
    functions = get_functions(ollama_output["response"])
    rospy.loginfo("Parsed functions: %s" % str(functions))
    for func_str in functions:
        rospy.loginfo("Attempting generated function: " + func_str + ":")
        try:
            exec(func_str)
        except Exception as e:
            succeeded = False
        else:
            succeeded = True

    return ollama_output, succeeded

def handle_ollama_call(req):
    global ollama_intention
    start_time = time.time()
    ollama_confirm_pub = rospy.Publisher("/ollama_response", OllamaResponse, queue_size = 1)

    rospy.loginfo("Recieved ollama request '%s'" % req.input)

    ultimately_failed = True
    for attempt_no in range(ollama_max_fails + 1):
        o, succeeded = main(req.input)
        if succeeded:
            ollama_confirm_pub.publish(success = True, intent = ollama_intention)
            ultimately_failed = False
            break

    if ultimately_failed:
        ollama_confirm_pub.publish(success = False, intent = ollama_intention)
        rospy.loginfo("Ollama ultimately failed afted %d retries" % ollama_max_fails)
    else:
        rospy.loginfo("Ollama succeeded")

    time_taken = time.time() - start_time
    rospy.loginfo("Had %d attempts and took %.1f seconds" % (attempt_no + 1, time_taken))
    # with open(os.path.join(os.path.dirname(__file__), "..", "ollama_benchmarks.csv"), "a") as f:
    #     f.write("%s,%.1f,%d,%s,%s,%d\n" % (platform.node(), time_taken, attempt_no + 1, ollama_intention, str(not ultimately_failed), len(req.input)))

    # print(o.keys())
    return OllamaCallResponse(
        o["total_duration"],
        o["load_duration"],
        o["prompt_eval_duration"],
        o["eval_count"],
        o["eval_duration"]
    )

def planner_intention_sub_cb(intention):
    global ollama_intention
    rospy.loginfo("Intention has been set to %s" % intention.data)
    ollama_intention = intention.data

if __name__ == "__main__":
    rospy.init_node("ollama_wrapper")
    rospy.Subscriber("/planner_intention", String, planner_intention_sub_cb)
    s = rospy.Service("/stt/ollamacall", OllamaCall, handle_ollama_call)
    print("Node started correctly")
    rospy.spin()
 


