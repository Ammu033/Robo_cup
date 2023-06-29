#!/bin/bash

echo "starting Chromium"

while true; do
  # kill previous chromium
  killall -9 firefox 2>/dev/null;

  #launch unclutter
 # unclutter &

  # launch browser
  DISPLAY=:0 firefox --private http://localhost:8000/index.html

done;
