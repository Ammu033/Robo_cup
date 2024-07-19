#!/bin/bash

# this ought to be added to the Dockerfile

apt-get install -y portaudio19-dev python-all-dev python3-all-dev && pip3 install pyaudio 
pip3 install jinja2 ollama geocoder requests python-dotenv parsimonious SpeechRecognition

chmod 777 /home/lcastor/ros_ws/src/LCASTOR/ollamawrapper/ollama_benchmarks.csv

