#!/usr/bin/env bash

apt update && apt install --no-install-recommends -y $(cat requirements.txt)
apt update && apt install --no-install-recommends -y $(cat requirements_ros.txt)
