# Turtle Playground

A ROS2 playground based on Turtlesim.

I use this to practice and try ideas out. Possibly buggy.

## Installation

### Dependencies

``` bash
sudo apt update
sudo apt install ros-kilted-desktop ros-kilted-turtlesim-msgs
```

There are also some extra deps not available as a debian package, so you need to manually build and install them:

``` bash
cd ~/workspace/ros
git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git
cd BehaviorTree.ROS2 ; git checkout humble
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
cd ~/workspace/ros
```

TODO: make this less cumbersome.

### Build

``` bash
cd turtle_playground
mkdir install
cp -r ../BehaviorTree.ROS2/install/b* install/
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

## Ideas

See [IDEAS.md](IDEAS.md).
