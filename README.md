# Linear transformations for pose calculation

## About

todo

## Building the workspace

It is recommended to use the provided docker setup. For instructions on how to build and run the dockerized environment, valid for both Windows and Linux based systems, take a look at [`/docker/README.md`](https://github.com/b-Tomas/robot-kinematics/blob/main/docker/README.md).

I case you want to run the project in a different way, make sure you meet the following requirements:
* Ubuntu 20.04
* ROS Noetic Ninjemys
* All of the dependencies listed in `docker/requirements.txt` are installed

Once all system dependencies are met, from within the container navigate into the `openmanipulator_x` folder and clone the OpenManipulator repos using `vsctool`:

```sh
cd ~/ws/src/robot-transformations/openmanipulator_x
vcs import < robotis.repos
```

And then build the project:

```sh
cd ~/ws/
catkin_make
```

Source the new overlay (remeber to run this step in every new bash session you use for interacting with these packages):

```sh
cd ~/ws/
. devel/setup.bash
```

To make sure everything is working, run the tests:

```sh
TODO: Tests? What tests?
```

## Running the code

You can use the provided launchfiles to run the simulation and the transformations server, and in a different tmux pane the CLI client:
```sh
# Launch simulation, GUI and transformations server
roslaunch openmanipulator_transformations transformations.launch
# Launch the CLI
roslaunch openmanipulator_transformations cli.launch
```

For OpenManipulator-only launches, you can use the original launchfiles:

```sh
# Simulation
roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch
# RViz visualization
roslaunch open_manipulator_description open_manipulator_rviz.launch
# Robot controller
roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false # Optional: use_moveit:=true
# User interfaces
roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch
roslaunch open_manipulator_teleop open_manipulator_teleop_keyboard.launch
```

And then run our nodes with:

```sh
# Math service
rosrun openmanipulator_transformations transformation_service.py
# CLI client
rosrun openmanipulator_transformations transformation_client.py
```
