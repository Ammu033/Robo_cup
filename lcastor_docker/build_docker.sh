#!/usr/bin/env bash

image_name=lcastor_base

docker build -t ${image_name} $(dirname "$0")/