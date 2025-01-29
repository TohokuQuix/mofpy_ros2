#!/usr/bin/env bash

set -ex

pre-commit install

echo "source /opt/ros/$ROS_DISTRO/setup.bash" >>~/.bashrc

sudo rosdep init
just deps

colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

pipx inject rosdoc2 sphinxcontrib-mermaid

echo "Done!"
