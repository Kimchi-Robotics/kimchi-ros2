
# Andino Navigation

<!-- <img src="docs/andino_gazebo.png"> -->

This package contains launch file required to make Andino 

## Build

Install package dependencies:

```
rosdep install --from-paths src -i -y
```

Build the package:

```
colcon build
```

Note: `--symlink-install` can be added if needed.

Finally, source the install folder
```
. install/setup.bash
```

# Usage

This package has the next option to be executed.

```
ros2 launch andino_gazebo andino_gazebo_slam.launch.py world:=hq_world.sdf
```

