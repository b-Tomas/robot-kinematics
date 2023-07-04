# Linear transformations for pose calculation

## About

todo

## Docker setup

- NVIDIA GPU support - *Skip this step if you don't have an NVIDIA graphics card*

Make sure you have the drivers installed:

```sh
nvidia-smi
```

Install `nvidia-container-toolkit` in your host machine:

```sh
sudo apt install -y nvidia-container-toolkit
```

- Build the docker image whose name is `ros_noetic_arm`:

```sh
./docker/build.sh
```

You can also try to set a specific image name:

```sh
./docker/build.sh -i my_fancy_image_name
```

- Run a docker container from `ros_noetic_arm` called `ros_noetic_container_arm`:

```sh
./docker/run.sh
```

You can also try to set specific image and container names:

```sh
./docker/run.sh -i my_fancy_image_name -c my_fancy_container_name
```

And a prompt in the docker image should appear at the root of the workspace:

```sh
$ pwd
/home/username/ws
```

After this initial setup, the OpenManipulator repositories must be cloned. More on that in the next section.

## Prepare your workspace

Once all system dependencies are met, from within the container navigate into the `openmanipulator_x` folder and clone the repos using `vsctool`:

```sh
cd ~/ws/src/robot-transformations/openmanipulator_x
vcs import < robotis.repos
```

And then build the project:
```sh
cd ~/ws/
catkin_make
```

To make sure everything is working, run the tests:
```sh
TODO: We don't even have any code yet
```

## Run the code

TODO: Create a launcfile for all of the required launches + our code

```sh
roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch
roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false # optional: use_moveit:=true
roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch
roslaunch open_manipulator_teleop open_manipulator_teleop_keyboard.launch
```