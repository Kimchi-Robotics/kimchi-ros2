## ROS2 jazzy guidelines

## Build

### Docker

#### Prerequisites

It is a requirement to have `docker engine` already installed in the host machine.

* See [Docker Installation Guide](https://docs.docker.com/engine/install/ubuntu/)

Also, is important to be able to manage docker with non-root users:
* See [Manage Docker as a non-root user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)

For NVIDIA GPU support, `nvidia-container-toolkit` should be installed. *Skip this step if you don't have an NVIDIA graphics card*


* Make sure you have the drivers installed:
  ```sh
  nvidia-smi
  ```
* See [NVIDIA Container Toolkit Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#docker)

#### Building image and running container

- Build the docker image whose name is `ros2_jazzy`:

```sh
./docker/build.sh
```

You can also try to set a specific image name:

```sh
./docker/build.sh -i my_fancy_image_name
```

- Run a docker container from `ros2_jazzy` called `ros2_jazzy_container`:

```sh
./docker/run.sh
```

- **IMPORTANT**: If you are using nvidia drivers add the `--use_nvidia` flag:

```sh
./docker/run.sh --use_nvidia
```

You can also try to set specific image and container names:

```sh
./docker/run.sh -i my_fancy_image_name -c my_fancy_container_name
```

- Inside the container, install dependencies via `rosdep`:

  ```sh
  rosdep install -i -y --rosdistro jazzy --from-paths src
  ```

Note that the repository is mounted into a workspace. That is convenient if you
are working in a single repository project. Note that for multi-repository
workspace you should use another tool like vcs-tool to control via a `.repos`
file the repositories in your workspace.

- To build:

  ```sh
  colcon build
  ```

If you reach this point and before moving to the next section you might want to exit the container and save it, so you don't need to install all the dependencies again.
For doing so simply `exit` the container and when doing so a prompt will ask you to overwrite the image with the current state of the container. In this occasion you will need to
do accept this writing `yes`.

## Running Kimchi simulation

After having built the repository, you can try launching Kimchi!

- Run:

```sh
source install/setup.bash
ros2 launch andino_gz andino_gz.launch.py nav2:=False rviz:=False world_name:=populated_office.sdf
```

**Note**: Remember you can check all the arguments that a launch file accepts by adding `-s` to the command: `ros2 launch andino_gz andino_gz.launch.py -s`.

### Bring up kimchi

On real robot:
```
ros2 launch kimchi_state kimchi_bring_up.launch.py
```

Simulation:

```
ros2 launch kimchi_state kimchi_bring_up.launch.py use_sim_time:=true
```


### Slam

```
ros2 launch kimchi_navigation kimchi_slam.launch.py use_sim_time:=true
```

### Navigation

```
ros2 launch kimchi_navigation kimchi_navigation.launch.py
```

### Run gRPC server

```
ros2 launch kimchi_grpc_server kimchi_grpc_server.launch.py
```

### Test different planner and controllers combinations

There's a launch file within kimchi_navigation that is meant for testing planners and controller. If works both for the real robot and the simulation.

**Prerequisites**: 
- Have a map created in [kimchi_navigation/maps](https://github.com/Kimchi-Robotics/kimchi-ros2/tree/main/kimchi_navigation/maps)
- Bring up kimchi. Either real or simulation.

You can run it like (Remember to set `use_sim_time` accordingly)

```
ros2 launch kimchi_navigation kimchi_navigation.launch.py use_sim_time:=True nav_planner:=smac_lattice nav_controller:=mppi
```

You can check supported planners and controllers by checking the launch options like:

```
ros2 launch kimchi_navigation kimchi_navigation.launch.py -s
```

### Generate Protocol buffers

cd ./kimchi_grpc_server/kimchi_grpc_server/proto
./1_prepare_venv.sh
./2_gen_grpc_and_protobuf.sh


### Teleop

While running the simulation open another terminal and run

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/cmd_vel
```
For teleoperating Andino with the keyboard.

