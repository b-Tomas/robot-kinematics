# ekuabc

## ROS2 Humble guidelines

Base project docker image with ros2 humble full distribution and a sample package.

## CI

CI relies on two Github Action packages that essentially configures the ROS2
Humble environment to build and test the packages. If extra dependencies are
required which cannot be handled by `rosdep` you must perform the custom
installation steps before the execution of `action-ros-ci`.

## Docker

- NVIDIA GPU support - *Skip this step if you don't have an NVIDIA graphics card*

Make sure you have the drivers installed:

```sh
nvidia-smi
```

Install `nvidia-container-toolkit` in your host machine:

```sh
sudo apt install -y nvidia-container-toolkit
```

- Build the docker image whose name is `ros2_humble`:

```sh
./docker/build.sh
```

You can also try to set a specific image name:

```sh
./docker/build.sh -i my_fancy_image_name
```

- Run a docker container from `ros2_humble` called `ros2_humble_container`:

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
$ ls -lash src/
total 12K
4.0K drwxr-xr-x 3 root         root         4.0K Nov 25 20:53 .
4.0K drwxr-xr-x 1 root         root         4.0K Nov 25 20:53 ..
4.0K drwxrwxr-x 4 agalbachicar agalbachicar 4.0K Nov 25 19:20 ekuabc
```

Note that the repository is mounted into a workspace. That is convenient if you
are working in a single repository project. Note that for multi-repository
workspace you should use another tool like vcs-tool to control via a `.repos`
file the repositories in your workspace.

## Prepare your workspace, build and test

- To install dependencies via `rosdep`:

```sh
rosdep install -i -y --rosdistro humble --from-paths src
```
- To build:

```sh
colcon build
```

And if you want details of the run commands:

```sh
colcon build --event-handlers console_direct+
```

- To test:

```sh
colcon test
# You should also consider to add --event-handlers console_direct+ to better
# understand what's going on:
# $ colcon test --event-handlers console_direct+
colcon test-result
```

## Try the code!

This is based on the [pub-sub Python tutorial](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
and the [pub-sub C++ tutorial](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
so consider looking at the specific instructions in the page for all the details.

Once you have finished building and testing your workspace, make sure you try it.

- Open tmux:

```sh
tmux
```

- Open two panes by sequentially pressing `Ctrl-b` and then `"`.

- Source your install space:

```sh
source install/setup.bash
```

- In one pane run:

```sh
ros2 run py_pubsub talker
```

Alternatively you could try the C++ `talker` by doing:

```sh
ros2 run cpp_pubsub talker
```

- And in the other (you can switch between panes by sequentially pressing
  `Ctrl-b` and the `up` and `down` arrow keys):

```sh
ros2 run py_pubsub listener
```

Alternatively you could try the C++ `listener` by doing:

```sh
ros2 run cpp_pubsub listener
```

You should see that in the `talker` pane you get logs every time a message is
sent with an increasing counter and in the `listener` pane you get the sent
message right after it was logged in the other one.

You can stop each node by pressing `Ctrl-c` and then exit each tmux pane by
`exit`ing the terminal session. You should return to the initial bash session
in the container.
