#!/bin/bash

cd $1/www

echo "starting http.server"
python3 -m http.server

#chrome --kiosk http://localhost:8000/index.html  # not easily possible to minimize chrome without having to close it 
