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

DEFAULT_OLLAMA_URL = "http://192.168.69.34:11434"
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
        You never need to call the publish_what_im_doing() function. \
        You may only use the functions provided to you in the context. Do not use print(str) but use engine_say(p, str) instead. \
        The engine_say(p, str) also speaks out loud. You need to call the functions I gave you to complete the task. \
        You may not write any code other than with the functions that have been defined. \
        For example, the task 'tell me what is the heaviest object on the sink' could call: \
        goto_location(p, location_name='sink') then identify_objects(p, what_to_idenfify='the heaviest object') \
        then go_back_to_me(p) then report_information(p). Another example is that 'lead Robin from the dinner table to the hallway' \
        could call `goto_location(p, location_name='sink')` then `ask_for_person(p, person_name='Robin'), then` \
        `guide_person(p)` and then finally `goto_location(p, location_name='hallway')`" % req.input
      
        response = self.query_engine.query(prompt).response
        print(response)
        return GPSRRAGCallResponse(
            response,
            "",
            time.time() - starttime
        )

if __name__ == "__main__":
    rospy.init_node("gpsr_rag_node")
    gpsr = GPSRRagNode()