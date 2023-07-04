# Download base image ROS Noetic Ninjemys (Ubuntu Focal)
FROM osrf/ros:noetic-desktop-full

# Arguments for building
ARG USERID
ARG USER

# Image name
ARG "Simulation and testing environment for the OpenManipulator robot arm"

# Setup environment
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Copy installation scripts, and perform the dependency installation.
COPY docker/install.sh .
COPY docker/requirements.txt .
COPY docker/requirements_ros.txt .
RUN ./install.sh
RUN rm install.sh requirements.txt requirements_ros.txt

# Create a user with passwordless sudo
RUN adduser --uid $USERID --gecos "unlp student" --disabled-password $USER
RUN adduser $USER sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/$USER/.bashrc
USER $USER

# Creates the src folder of the workspace
RUN mkdir -p /home/$USER/ws/src

# Adds to .bashrc the ros noetic overlay sourcing
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/$USER/.bashrc

# Defines a workspace folder
WORKDIR /home/$USER/ws

CMD ["/bin/bash"]
