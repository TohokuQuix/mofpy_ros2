#!/usr/bin/env bash

set -ex

# ROS環境をセットアップ
# 既にインストールされていたら実行しない
if command -v ros2 >/dev/null 2>&1; then
    echo "ROS2 command found. Skipping ROS installation"
else
    echo "ROS2 not found. Install ROS ..."
    sudo apt update
    sudo apt upgrade -y
    sudo apt install software-properties-common curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    sudo apt update
    sudo apt install ros-jazzy-desktop
    echo "source /opt/ros/jazzy/setup.bash" >>~/.bashrc
fi

# aptでインストール可能なパッケージをインストール
sudo apt install python3-colcon-mixin mold ccache cpplint clang-format cmake-format doxygen python3-rosdep yamllint npm just pipx
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

# pipxでインストール可能なパッケージをインストール
pipx install rosdoc2 pre-commit
pipx inject rosdoc2 sphinxcontrib-mermaid

# 依存関係の解消
just deps
