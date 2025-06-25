
# Kimchi Navigation

<img src="docs/kimchi_robot.png">

This package contains launch file required to make Kimchi navigate.

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
ros2 launch kimchi_navigation kimchi_slam.launch.py
```

