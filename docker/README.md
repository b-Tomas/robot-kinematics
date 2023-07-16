<!--
Based on https://github.com/Ekumen-OS/andino/blob/5236b1132343539e19c2eb63c61e5f5f7814555b/docker/README.md
From Ekumen-OS/andino project
Copyright (c) 2023, Ekumen Inc., under BSD 3-Clause License
-->

# Docker setup

## Prerequisites

It is a requirement to have `docker engine` already installed in the host machine.

- See [Docker Installation Guide](https://docs.docker.com/engine/install/) for your preferred platform

### Running on Linux

For NVIDIA GPU support, `nvidia-container-toolkit` should be installed. _Skip this step if you don't have an NVIDIA graphics card_

- Make sure you have the drivers installed:
  ```sh
  nvidia-smi
  ```
- See [NVIDIA Container Toolkit Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

- Build the docker image whose default name is `ros_noetic_arm`:

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

- **IMPORTANT**: If you are using nvidia drivers add the `--use_nvidia` flag:

```sh
./docker/run.sh --use_nvidia
```

You can also try to set specific image and container names:

```sh
./docker/run.sh --use_nvidia -i my_fancy_image_name -c my_fancy_container_name
```

And a prompt in the docker image should appear at the root of the workspace:

```sh
$ pwd
/home/username/ws
```

After this initial setup, the OpenManipulator repositories must be cloned and the project must be built. More on that in the root [`README.md`](https://github.com/b-Tomas/robot-kinematics/blob/main/README.md) file of the repository.


### Running on Windows

Assuming you have Docker Desktop/Engine installed, you might want to install WSL2 and use Docker with WSL2
instead of Hyper-V. This guide is written with that in mind.

You also need a X11 server, like [VcXsrc](https://sourceforge.net/projects/vcxsrv/). X11 will show the GUI of the
simulator on your Windows PC. When initializating it, make sure to "Disable access control" so WSL can use it. You
_maaay_ want to disable Clipboard support, for security reasons, but you can leave it if you like danger.

With the Docker Engine started and the X11 server running, open a new PowerShell terminal (version 7+ recommended),
go to the root of the repository and run `.\docker\build.ps1` to build and `.\docker\run.ps1` to run. Standard help
for those command can be accessed with `Get-Help` and `-?`.

After you've built the image and you are inside the container, you may prepare your workspace (next section).
