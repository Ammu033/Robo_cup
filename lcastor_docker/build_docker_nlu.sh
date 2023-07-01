#!/usr/bin/env bash

image_name=lcas.lincoln.ac.uk/lcastor/lcastor_base_nlu

docker build --build-arg UID=$(id -u) --build-arg GID=$(id -g) -t ${image_name}  - < Dockerfile_nlu
