# Turtle Playground

A ROS2 playground based on Turtlesim.

I use this to practice and try ideas out. Possibly buggy.

## Installation

### Dependencies

``` bash
sudo apt update
sudo apt install ros-kilted-desktop ros-kilted-turtlesim-msgs
```

### Build

``` bash
cd turtle_playground
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

## Ideas

See [IDEAS.md](IDEAS.md).
