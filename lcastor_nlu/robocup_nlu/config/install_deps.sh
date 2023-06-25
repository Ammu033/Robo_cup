#!/bin/bash

### INSTALL PREREQUISITES

# TMULE
pip install tmule
sudo apt install tmux
echo "Tmule installed"

# Speech Recognition 
pip install SpeechRecognition
echo "SpeechRecognition installed"

# PyAudio
sudo apt install python3-pyaudio
echo "PyAudio installed"

# Rospkg
pip install rospkg
echo "rospkg installed"

# Whisper
pip install git+https://github.com/openai/whisper.git
echo "Whisper installed"

# Torch
#pip install --extra-index-url https://download.pytorch.org/whl/cu113
pip install torch
echo "Torch installed"

# RASA
pip3 install rasa==3.5.11
pip3 install pyOpenSSL --upgrade
pip install netifaces
echo "Rasa installed"

# FLASK
pip3 install flask
echo "Flask installed"
