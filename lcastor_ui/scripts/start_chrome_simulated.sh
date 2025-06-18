#!/bin/bash

echo "starting Chromium"
DISPLAY=:0 chromium-browser --start-fullscreen http://localhost:8000/index.html 
