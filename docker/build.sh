#!/bin/bash

# Based on https://github.com/Ekumen-OS/andino/blob/8a58d232fc60f6f241a57a012a6a2e300b8b5dcf/docker/build.sh
# From Ekumen-OS/andino project
# Copyright (c) 2023, Ekumen Inc., under BSD 3-Clause License

set -e

# Prints information about usage.
function show_help() {
  echo $'\nUsage:\t build.sh [OPTIONS] \n
  Options:\n
  \t-i --image_name\t\t Name of the image to be built (default ros_noetic_arm).\n
  Example:\n
  \tbuild.sh --image_name custom_image_name\n'
}

echo "Building the docker image for ros noetic development."

SCRIPT_FOLDER_PATH="$(cd "$(dirname "$0")"; pwd)"
CONTEXT_FOLDER_PATH="$(cd "$(dirname "$0")"; cd .. ; pwd)"

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -i|--image_name) IMAGE_NAME="${2}"; shift ;;
        -h|--help) show_help ; exit 1 ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Update the arguments to default values if needed.
IMAGE_NAME=${IMAGE_NAME:-ros_noetic_arm}
DOCKERFILE_PATH=$SCRIPT_FOLDER_PATH/Dockerfile

USERID=$(id -u)
USER=$(whoami)

docker build -t $IMAGE_NAME \
     --file $DOCKERFILE_PATH \
     --build-arg USERID=$USERID \
     --build-arg USER=$USER \
     $CONTEXT_FOLDER_PATH
