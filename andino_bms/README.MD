
# Andino BMS

Mock for battery management

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

## Run

```
ros2 run andino_bms andino_bms
```