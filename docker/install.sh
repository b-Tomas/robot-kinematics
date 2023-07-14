#!/usr/bin/env bash

apt update && apt install --no-install-recommends -y $(cat requirements.txt)
