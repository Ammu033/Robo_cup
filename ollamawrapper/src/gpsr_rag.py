#!/usr/bin/env python3

__import__('pysqlite3')
import sys
sys.modules['sqlite3'] = sys.modules.pop('pysqlite3')

from llama_index.core import VectorStoreIndex, SimpleDirectoryReader, Settings, Document
from llama_index.embeddings.huggingface import HuggingFaceEmbedding
from llama_index.llms.ollama import Ollama
from llama_index.readers.file import MarkdownReader
from llama_index.readers.database import DatabaseReader
from llama_index.vector_stores.chroma import ChromaVectorStore
from llama_index.core import StorageContext
from dataclasses import dataclass
from ollamamessages.srv import GPSRRAGCall, GPSRRAGCallResponse
from std_msgs.msg import String
import subprocess
import platform
import inspect
import tempfile
import chromadb
import pandas
import typing
import jinja2
import ollama
import rospy
import time
import ast
import os
import re

DEFAULT_OLLAMA_URL = "http://192.168.69.54:11434"
DEFAULT_OLLAMA_MODEL = "llama3.1"

sys.path.insert(1, os.path.join(os.path.dirname(__file__), "..", "..", "lcastor_actions"))
import gotoRoom

sys.path.insert(2, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from make_vector_embeddings import BASE_INFO_PATH, EMBEDDINGS_MODEL, EMBEDDINGS_PATH

import capabilities

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import time
import pnp_cmd_ros
from pnp_cmd_ros import *

class GPSRRagNode:
    def __init__(self):
        Settings.embed_model = EMBEDDINGS_MODEL
        Settings.llm = Ollama(
            base_url = rospy.get_param("/gpsr/rag/ollama_api_url", DEFAULT_OLLAMA_URL), 
            model = rospy.get_param("/gpsr/rag/ollama_decomposition_model", DEFAULT_OLLAMA_MODEL), 
            request_timeout = 360.0
        )

        db = chromadb.PersistentClient(path = EMBEDDINGS_PATH)
        chroma_collection = db.get_or_create_collection(BASE_INFO_PATH)
        vector_store = ChromaVectorStore(chroma_collection = chroma_collection)
        storage_context = StorageContext.from_defaults(vector_store = vector_store)

        index = VectorStoreIndex.from_vector_store(
            vector_store, storage_context = storage_context
        )

        self.query_engine = index.as_query_engine()

        s = rospy.Service("/gpsr/rag/task_decomposition", GPSRRAGCall, self.handle_decomposition_call)
        print("Node started correctly")
        rospy.spin()

    def handle_decomposition_call(self, req):
        starttime = time.time()
        rospy.loginfo("Recieved new decompostion call '%s'" % req.input)

        prompt = \
        "You are an AI agent controlling a home assistance robot. You have been provided with a context \
        of all of the avaliable items and their locations, all of the locations and rooms and the names of \
        the people in the scene. You have recieved a request from a human to '%s'. \
        You need to create a function for completing the task. \
        The function signature must be: do_func(p)->None. You cannot write anything outside the function. \
        You need to use the functions with which you have been provided to do this. \
        The first parameter of these functions should always be `p`. You don't need to worry about its definition. \
        You can only go to locations with which you have been provided. \
        If you are asked to go to a location or room that doesn't exist, say so. \
        You do not need to def the functions in the context. \
        You do not need to explain your code. \
        You may only use the functions provided to you in the context. Do not use print(str) but use engine_say(p, str) instead. \
        The engine_say(p, str) also speaks out loud. You need to call the functions I gave you to complete the task. \
        You may not write any code other than with the functions that have been defined. \
        You may not define any variables. \
        For example, the task 'tell me what is the heaviest object on the sink' could call: \
        identify_objects(p, object_location='sink', what_to_idenfify='the heaviest object') \
        then go_back_to_me(p) then report_information(p). Another example is that 'lead Robin from the dinner table to the hallway' \
        could call `goto_location(p, location_name='dinner table')` then \
        `guide_person(p, to_location='hallway', person_name='Robin')`" % req.input

        while True:     
            func_str_raw = self.query_engine.query(prompt).response
            valid, invalid_message = self.validate_func(func_str_raw)
            if valid:
                break
            else:
                rospy.loginfo("Retrying due to reason '%s'" % invalid_message)
                prompt += "\n" + invalid_message

        s = "=== The generated plan is the following: ==="
        print(s)
        print(func_str_raw)
        print("=" * len(s)) 

        environment = jinja2.Environment(loader = jinja2.FileSystemLoader(os.path.dirname(__file__)))
        template = environment.get_template("gpsr_runner.py.jinja2")
        with tempfile.NamedTemporaryFile(suffix = ".py", mode = "w", delete = False) as f:
            f.write(template.render(
                cwd = os.path.join(os.path.dirname(__file__), "capabilities"), 
                main_func = func_str_raw, 
                func_call = "do_func(p = p)")
            )
            filename = f.name
            rospy.loginfo(str(filename))

        stdout = ""
        stderr = ""
        try:
            proc = subprocess.Popen(["python3", filename], stdout = subprocess.PIPE, stderr = subprocess.PIPE)
            while True:
                line = proc.stdout.readline()
                if not line:
                    break
                stdout += line.decode()
                rospy.loginfo("[stdout] %s" % line.rstrip().decode())
            while True:
                line = proc.stderr.readline()
                if not line:
                    break
                stderr += line.decode()
                rospy.logwarn("[stderr] %s" % line.rstrip().decode())
        except KeyboardInterrupt:
            proc.terminate()
            rospy.loginfo("Proc terminated")

        if stderr == "" and not "!! Exception" in stdout:
            # all good, the plan was executable and run without errors
            rospy.loginfo("Global plan succeeded")
   
        return GPSRRAGCallResponse(
            func_str_raw,
            filename,
            time.time() - starttime
        )
    
    def validate_func(self, func_str):
        asked_for_person = False

        if "for " in func_str:
            return False, "You are not allowed to use for loops."
        if "while " in func_str:
            return False, "You are not allowed to use while loops."
        if "if " in func_str:
            return False, "You are not allowed to use if statements."
        if " = " in func_str:
            return False, "You are not allowed to define any variables."
        
        try:
            body = ast.parse(func_str).body[0].body
        except IndexError:
            return False, "The function was not declared properly."

        for expression in body:
            if type(expression.value) is ast.Call:
                func_name = expression.value.func.id
                print("Called '%s()'" % func_name)  
                
                if func_name == "ask_for_person":
                    asked_for_person = True

                if func_name == "offer_object_to_person":
                    for k in expression.value.keywords:
                        if k.arg == "person_name" and k.value.value.lower() in ["me", "the human", "you", "human"]:
                            return False, "Instead of calling `offer_object_to_person()` with 'me' as a parameter you should use the function `offer_object_to_me()`."

                if func_name == "answer_quiz" and not asked_for_person:
                    return False, "You need to call `ask_for_person()` before answering a quiz."
        
        return True, True

if __name__ == "__main__":
    rospy.init_node("gpsr_rag_node")
    gpsr = GPSRRagNode()