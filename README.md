# mofpy_ros2

<div style="margin: 0 auto" >
    <img src="docs/logo.png" alt="Logo of mofpy" />
</div>

## What is mofpy

mofpyは，ジョイパッドの入力から汎用的にアクションを実行できるROS 2ノードです．

ROS1版は，[こちら](https://github.com/naoki-mizuno/mofpy/tree/master)にあり，これをベースにROS 2用に開発したものがこちらのリポジトリになります．

<!-- ここにMofpyの動作イメージ図を入れる -->

## Continue Integration Status

[![CI (Jazzy)](https://github.com/KazuyaOguma18/mofpy_ros2/actions/workflows/build-and-test.yaml/badge.svg?branch=main)](https://github.com/KazuyaOguma18/mofpy_ros2/actions/workflows/build-and-test.yaml?query=branch%3Amain)

## Getting Started

### Installation

ROS 2のインストールが完了していることを前提とします．
ROS 2インストール手順は[こちら](https://docs.ros.org/en/jazzy/Installation.html)を参照してください．

```bash
git clone https://github.com/KazuyaOguma18/mofpy_ros2.git
bash ./setup.bash
just build
```

### Usage

```bash
source install/setup.bash
ros2 launch mofpy_demo mofpy_demo.launch.py
```

## Packages

### mofpy

mofpyのROS 2ノードが含まれています．
実装はすべてこのパッケージ内に記述されています．

### mofpy_demo

mofpyのデモ用のパッケージです．
launchファイルや設定ファイルが含まれています．

### moveit_py_configs_utils

moveit_setup_assistantで生成されたMoveItの設定ファイルをMoveItPyで読み込むためのユーティリティパッケージです．
[moveit_configs_utils](https://github.com/moveit/moveit2/tree/main/moveit_configs_utils)を一部改変したものです．

## Supported Joypads

- [x] DualShock (PS4)
- [ ] DualSense (PS5) comming soon ...

## Supported Actions

[mofpy/mofpy/action](https://github.com/KazuyaOguma18/mofpy_ros2/tree/main/mofpy/mofpy/action) にこれらの実装があります

| Action              | Description                            |
| ------------------- | -------------------------------------- |
| moveit_named_target | MoveItの名前付きターゲットを実行します |
| moveit_servo_joint  | MoveIt ServoのFKを実行します           |
| moveit_servo_twist  | MoveIt ServoのIKを実行します           |
| publish             | 任意のトピックにメッセージを送信します |
| shared_list         | 配列型の共有変数を操作します           |
| shared_value        | スカラー型の共有変数を操作します       |
