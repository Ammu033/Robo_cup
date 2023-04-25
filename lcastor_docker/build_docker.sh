#!/usr/bin/env bash

image_name=lcas.lincoln.ac.uk/lcastor/lcastor_base

docker build -t ${image_name} $(dirname "$0")/
