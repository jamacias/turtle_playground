# Turtle Playground

A ROS2 playground based on Turtlesim.

I use this to practice and try ideas out. Possibly buggy.

## Installation

### Dependencies

``` bash
sudo apt update
sudo apt install ros-kilted-desktop ros-kilted-turtlesim-msgs python3-vcs2l
source /usr/share/vcs2l-completion/vcs.bash # or add it to ~/.bashrc
```

### Build

``` bash
git clone git@github.com:jamacias/turtle_playground.git
cd turtle_playground
vcs import --input turtle_playground.repos
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

## Ideas

See [IDEAS.md](IDEAS.md).
