#!/usr/bin/env bash

apt update && apt install --no-install-recommends -y $(cat requirements.txt)

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list | \
    tee /etc/apt/sources.list.d/ros2.list > /dev/null
sed -i -e 's/ubuntu .* main/ubuntu jammy main/g' /etc/apt/sources.list.d/ros2.list

apt update && apt install --no-install-recommends -y $(cat requirements_ros.txt)
