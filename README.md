## ROS2 Humble guidelines

Base project docker image with ros humble full distribution and a sample package.

### CI

CI relies on two Github Action packages that essentially configure the ROS
Humble environment to build and test the packages. If extra dependencies are
required which cannot be handled by `rosdep` you must perform the custom
installation steps before the execution of `action-ros-ci`.

Additionally, pre-commit runs to check format and some other nits. Check the section `Using pre-commit` below
to learn how to run it locally.

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
  rosdep install -i -y --rosdistro humble --from-paths src
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

## Running Andino simulation

After having built the repository, you can try launching Andino!

- Run:

```sh
source install/setup.bash
ros2 launch andino_gazebo andino_one_robot.launch.py
```

This launch file accepts passing an argument for opening `rviz` automatically.

`ros2 launch andino_gazebo andino_one_robot.launch.py rviz:=true`

**Note**: Remember you can check all the arguments that a launch file accepts by adding `-s` to the command: `ros2 launch andino_gazebo andino_one_robot.launch.py -s`.

### Slam

```
ros2 launch andino_gazebo andino_gazebo_slam.launch.py world:=hq_world.sdf
```

### Navigation

```
ros2 launch andino_gazebo andino_gazebo_navigation.launch.py
```

### Run gRPC server

```
ros2 launch kimchi_grpc_server kimchi_grpc_server.launch.py
```

### Generate Protocol buffers

cd ./kimchi_grpc_server/kimchi_grpc_server/proto
./1_prepare_venv.sh
./2_gen_grpc_and_protobuf.sh

### Running Andino with Battery

```sh
source install/setup.bash
ros2 launch andino_gazebo andino_one_robot.launch.py  use_andino_w_battery:=true
```

### Inspect topics

While running the simulation open another terminal and run

```sh
ros2 topic list
```
For checking the available topics.

### Teleop

While running the simulation open another terminal and run

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/cmd_vel
```
For teleoperating Andino with the keyboard.

### Using colcon 101

See [Using colcon to build packages](https://docs.ros.org/en/humble/Tutorials/Colcon-Tutorial.html#using-colcon-to-build-packages) or directly [colcon website](https://colcon.readthedocs.io/en/released/index.html) for further information.

Use `colcon --help` for checking out available commands. Nevertheless here are some recommended commands:
- To build/test the entire workspace.
  ```sh
  colcon build
  ```
  ```sh
  colcon test
  ```

- To build/test a particular package/packages.
  ```sh
  colcon build --packages-select <package_name>
  ```
  ```sh
  colcon test --packages-select <package_name>
  ```

- To build/test a particular package/packages and their dependencies.
  ```sh
  colcon build --packages-up-to <package_name>
  ```
  ```sh
  colcon test --packages-up-to <package_name>
  ```

- For printing information when building/testing add flag `--event-handlers=console_direct+`, e.g.:
  ```sh
  colcon build --packages-up-to <package_name> --event-handlers=console_direct+
  ```

- Use `colcon graph` or `colcon list` to get information about the packages and their dependencies in the workspace:
  ```sh
  colcon graph
  ```
  ```sh
  colcon list
  ```

## Try the example ROS2 code!

This is based on the [pub-sub Python tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#)
and the [pub-sub C++ tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
so consider looking at the specific instructions in the page for all the details.

Once you have finished building and testing your workspace, make sure you try it.

- Open tmux:

```sh
tmux
```

- Open two panes by sequentially pressing `Ctrl-b` and then `"`.

- Source your development space:

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

- In one pane run:

```sh
ros2 launch abc_gazebo_worlds launch_abc_world.launch.py
```

It will run Gazebo with the default sample world.

Alternatively, if you add new world files to the package, you can specify them by name:

```sh
ros2 launch abc_gazebo_worlds launch_abc_world.launch.py
abc_world_name:=my_fancy_custom_world
```

- In the other pane, try spawning the sample models (you can switch between panes by sequentially pressing
  `Ctrl-b` and the `up` and `down` arrow keys):

```sh
ros2 launch abc_sample_spawns spawn_robots.py
```

The model is spawned three times in the world, however, each one of them is described and loaded using a different format description: SDF, URDF, and XACRO.
Please refer to the [`launchfile`](tutorials/abc_sample_spawns/launch/spawn_robots.py) for further information.

## Dynamic world files

The package `abc_gazebo_worlds` contains a world file with some ERB code in it, along with a launch file specifically prepared to parse the template file before launching Gazebo.

Try it out running:
```sh
ros2 launch abc_gazebo_worlds launch_dynamic_abc_world.launch.py
```

You will see an empty world with 5 lamps, nothing that fancy. Close Gazebo and let's try this other command:
```sh
LAMPS=8 ros2 launch abc_gazebo_worlds launch_dynamic_abc_world.launch.py
```
The world should this time include 8 lamps as requested (alternatively, you can run `export LAMPS=8` before running ros2 launch alone).

This shows how worlds can be programmatically generated, and even modified through environment variables!

## Running rostest

There is also a simple test you can run to check the communication between Gazebo and ROS. Try it out running
```sh
colcon test abc_rostest_sample test-gazebo-ros.test
```

### Using pre-commit

Pre-commit is a tool that allows git's pre-commit hook integrate with various code linters and formatters.

To install `pre-commit`, run
```sh
pip install pre-commit
```

To automatically run it on each commit, from repository's root:
```sh
pre-commit install
```

And that's it! Every time you commit, `pre-commit` will trigger and let you know if everything goes well.
If the checks fail, the commit won't be created, and you'll have to fix the issue (some of them are automatically fixed by `pre-commit`), STAGE the changes, and try again.

To manually run `pre-commit` on the staged changes, one can run:
```sh
pre-commit run
```

Or to change the whole codebase
```sh
pre-commit run --all-files
```

**Note**: `pre-commit` only runs on staged changes by default.
**Note2**: To bypass `pre-commit`, use `git commit --no-verify`.
