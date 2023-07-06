# Linear transformations for pose calculation

## About

todo

## Docker setup

### Linux

- NVIDIA GPU support - _Skip this step if you don't have an NVIDIA graphics card_

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

### Windows

Assuming you have Docker Desktop/Engine installed, you might want to install WSL2 and use Docker with WSL2
instead of Hyper-V. This guide is written with that in mind.

You also need a X11 server, like [VcXsrc](https://sourceforge.net/projects/vcxsrv/). X11 will show the GUI of the
simulator on your Windows PC. When initializating it, make sure to "Disable access control" so WSL can use it. You
_maaay_ want to disable Clipboard support, for security reasons, but you can leave it if you like danger.

With the Docker Engine started and the X11 server running, open a new PowerShell terminal (version 7+ recommended),
go to the root of the repository and run `.\docker\build.ps1` to build and `.\docker\run.ps1` to run. Standard help
for those command can be accessed with `Get-Help` and `-?`.

After you've built the image and you are inside the container, you may prepare your workspace (next section).

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

Source the new overlay (remeber to run this step in every new bash session you use for interacting with these packages):

```sh
cd ~/ws/
. devel/setup.bash
```

To make sure everything is working, run the tests:

```sh
TODO: We don't even have any code yet
```

## Run the code

You can use the provided launchfiles to run the simulation and transformation server, and in a different pane the CLI client
```sh
# Launch simulation, GUI and transformations server
roslaunch openmanipulator_transformations transformations.launch
# Launch the CLI
roslaunch openmanipulator_transformations cli.launch
```


-- deprecated --

```sh
roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch
roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false # optional: use_moveit:=true
roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch
roslaunch open_manipulator_teleop open_manipulator_teleop_keyboard.launch
```
