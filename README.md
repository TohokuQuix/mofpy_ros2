# mofpy_ros2

<div style="margin: 0 auto" >
    <img src="docs/logo.png" alt="Logo of mofpy" />
</div>

## 1. What is mofpy

mofpyは，ジョイパッドの入力から汎用的にアクションを実行できるROS 2ノードです．

ROS1版は，[こちら](https://github.com/naoki-mizuno/mofpy/tree/master)にあり，これをベースにROS 2用に開発したものがこちらのリポジトリになります．

<!-- ここにMofpyの動作イメージ図を入れる -->

## 2. Continue Integration Status

[![CI (Jazzy)](https://github.com/KazuyaOguma18/mofpy_ros2/actions/workflows/build-and-test.yaml/badge.svg?branch=main)](https://github.com/KazuyaOguma18/mofpy_ros2/actions/workflows/build-and-test.yaml?query=branch%3Amain)

## 3. Getting Started (Native)

### 3.1 Installation

ROS 2のインストールが完了していることを前提とします．
ROS 2インストール手順は[こちら](https://docs.ros.org/en/jazzy/Installation.html)を参照してください．

```bash
git clone https://github.com/KazuyaOguma18/mofpy_ros2.git
bash ./setup.bash
just build
```

### 3.2 Run

```bash
source install/setup.bash
ros2 launch mofpy_demo mofpy_demo.launch.py
```

## 4 Getting Started (DevContainer)

VSCodeの[Dev Containers拡張機能](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)を利用して開発することができます．

### 4.1 Install Docker

[公式サイト](https://docs.docker.com/engine/install/ubuntu/)からDockerをインストールしてください．

### 4.2 Install Dev Containers

VSCodeの拡張機能から[Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)をインストールしてください．

### 4.3 Open in Dev Container

コマンドパレット(Ctrl+Shift+P)から`Dev Containers: Reopen in Container`を選択し，Dev Containerを立ち上げてください．初回起動時にはしばらく時間がかかります．

### 4.4 Build

エラーなくDev Containerが立ち上がったら，以下のコマンドを実行してビルドしてください．

```bash
just build
```

### 4.5 Run

```bash
source install/setup.bash
ros2 launch mofpy_demo mofpy_demo.launch.py
```

## 5. Packages

### mofpy

mofpyのROS 2ノードが含まれています．
実装はすべてこのパッケージ内に記述されています．

### mofpy_demo

mofpyのデモ用のパッケージです．
launchファイルや設定ファイルが含まれています．

### moveit_py_configs_utils

moveit_setup_assistantで生成されたMoveItの設定ファイルをMoveItPyで読み込むためのユーティリティパッケージです．
[moveit_configs_utils](https://github.com/moveit/moveit2/tree/main/moveit_configs_utils)を一部改変したものです．

## 6. Supported Joypads

- [x] DualShock (PS4)
- [ ] DualSense (PS5) comming soon ...

## 7. Actions

ジョイパッドの入力をトリガーにして，汎用的なアクションを実行することができます．
アクションは，以下のようにpresetsの子アイテムとして定義されます．

基本的な構成として，アクション名をキーとして，トリガーとアクションのリストを持ちます．

トリガーには，ジョイパッドのボタンの組み合わせを指定します．
以下の例であれば，C_Uボタン(=上十字キー)が押されたときにアクションが実行されます．
複数のボタンを同時に押したときにアクションを実行したい場合は，リストにボタンを追加します．
長押しでアクションを実行したい場合は，リストの一番最後の値に長押しの時間を指定します．

```yaml
presets:
  sample:
    trigger: [C_U]
    action:
      - type: hoge
        ...
        # 任意のパラメータ
        ...
  long_press:
    trigger: [C_U, 1]
    action:
      - type: hoge
        ...
        # 任意のパラメータ
        ...
```

それらのアクションを組み合わせたものがこちらの例になります．それぞれのアクションについては，後述します．

```yaml
presets:
  switch_state_to_common:
    trigger: [OP, OP]
    action:
      - type: shared_value
        key: state
        initial: common
        value: common
  switch_state_to_arm:
    trigger: [SH, SH]
    action:
      - type: shared_value
        key: state
        value: arm
  switch_state_to_arm_fk:
    trigger: [[OP, SH], [OP, SH]]
    action:
      - type: shared_value
        key: state
        value: arm-fk
  state_pub:
    trigger: always
    action:
      - type: publish
        topic:
          name: state
          type: std_msgs/String
        values:
          data: ${shared(state)}
  super:
    trigger: [C_U, C_U, C_D, C_D, C_L, C_R, C_L, C_R, X, O]
    action:
      - type: publish
        topic:
          name: foo
          type: std_msgs/String
        values:
          data: You found the secret command!
  hello:
    trigger: always
    action:
      - type: publish
        topic:
          name: hello
          type: std_msgs/String
        values:
          data: Hello
  sample_twist:
    enabled_states: common
    trigger: always
    action:
      - type: publish
        topic:
          name: twist
          type: geometry_msgs/TwistStamped
        values:
          header:
            stamp: now
            frame_id: base_link
          twist:
            linear:
              x: ${axis(LSV) * 0.5}
            angular:
              z: ${axis(RSH) * 0.2}

  sample_pub_float_inc:
    enabled_states: common
    trigger: always
    action:
      - type: shared_value
        key: float_data
        step: 0.1
        enable_button: C_U
        initial: 0.5
      - type: publish
        topic:
          name: float
          type: std_msgs/Float32
        values:
          data: ${shared(float_data)}
```

### Supported Actions

[mofpy/mofpy/action](https://github.com/KazuyaOguma18/mofpy_ros2/tree/main/mofpy/mofpy/action) にこれらの実装があります

| Action                                                     | Description                             |
| ---------------------------------------------------------- | --------------------------------------- |
| [moveit_named_target](mofpy/README.md#moveit_named_target) | MoveGroupの名前付き目標姿勢を実行します |
| [moveit_servo_joint](mofpy/README.md#moveit_servo_joint)   | MoveIt ServoのFKを実行します            |
| [moveit_servo_twist](mofpy/README.md#moveit_servo_twist)   | MoveIt ServoのIKを実行します            |
| [publish](mofpy/README.md#publish)                         | 任意のトピックにメッセージを配信します  |
| [shared_list](mofpy/README.md#shared_list)                 | 配列型の共有変数を操作します            |
| [shared_value](mofpy/README.md#shared_value)               | スカラー型の共有変数を操作します        |
