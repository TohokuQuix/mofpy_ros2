# mofpy

## Actions

### flipper_effort_control

フリッパを力制御します

#### Parameters

| key             | type               | description
| --------------- | ------------------ | ----------------------
| namespace       | string             | コントローラの名前空間
| controller_name | string             | コントローラ名
| joints          | map(string, float) | ジョイント名と目標トルク値

#### Example

```yaml
  flipper_0deg:
    enabled_states: [chassis]
    trigger: Q
    action:
      - type: flipper_position_control
        controller_name: flipper_joint_position_controller
        joints:
          chassis/fl_flipper_joint: 0.
          chassis/fr_flipper_joint: 0.
          chassis/rl_flipper_joint: 0.
          chassis/rr_flipper_joint: 0.
```

### flipper_positon_control

フリッパを指定の位置へ制御します

#### Parameters

| key             | type               | description
| --------------- | ------------------ | ----------------------
| namespace       | string             | コントローラの名前空間
| controller_name | string             | コントローラ名
| joints          | map(string, float) | ジョイント名と目標位置

#### Example

```yaml
flipper_torque_low:
  enabled_states: [chassis]
  trigger: X
  action:
    - type: flipper_effort_control
      controller_name: flipper_joint_effort_controller
      joints:
        chassis/fl_flipper_joint: -2.0
        chassis/fr_flipper_joint: -2.0
        chassis/rl_flipper_joint: -2.0
        chassis/rr_flipper_joint: -2.0
```

### flipper_velocity_control

フリッパを速度制御します

#### Parameters

| key                                | type                | description
| ---------------------------------- | ------------------- | ----------------------
| namespace                          | string              | コントローラの名前空間
| controller_name                    | string              | コントローラ名
| scale                              | float               | 速度スケーリングかつ最大速度 [rad/s]
| mapping/synchronous/front_flippers | string[]            | 前フリッパの関節リスト
| mapping/synchronous/rear_flippers  | string[]            | 後フリッパの関節リスト
| mapping/synchronous/front/raise    | string              | 前フリッパを正方向に同期制御するjoyの軸名
| mapping/synchronous/front/lower    | string              | 前フリッパを負方向に同期制御するjoyの軸名
| mapping/synchronous/rear/raise     | string              | 後フリッパを正方向に同期制御するjoyの軸名
| mapping/synchronous/rear/lower     | string              | 後フリッパを負方向に同期制御するjoyの軸名
| mapping/independent/raise          | string              | フリッパを正方向に非同期制御するjoyの軸名
| mapping/independent/lower          | string              | フリッパを負方向に非同期制御するjoyの軸名
| mapping/independent/joints         | map(string, string) | 非同期制御する上でアクティブにするジョイント名とボタンのマッピング

#### Example

```yaml
flipper_velocity:
  enabled_states: [chassis]
  trigger: always
  action:
    - type: flipper_velocity_control
      controller_name: flipper_joint_velocity_controller
      scale: 0.5
      mapping:
        synchronous:
          front_flippers:
            - chassis/fl_flipper_joint
            - chassis/fr_flipper_joint
          rear_flippers:
            - chassis/rl_flipper_joint
            - chassis/rr_flipper_joint
          front:
            raise: R1
            lower: R2
          rear:
            raise: L1
            lower: L2
        independent:
          raise: C_U
          lower: C_D
          joints:
            chassis/fl_flipper_joint: L1
            chassis/fr_flipper_joint: R1
            chassis/rl_flipper_joint: L2
            chassis/rr_flipper_joint: R2

change_flipper_mode:
  trigger: [Q, 1]
  enabled_states: [chassis]
  action:
    - type: shared_list
      key: flipper_control_mode
      wrap: true
      values: [synchronous, independent]
```

### moveit_named_target

MoveGroupの名前付き目標姿勢を実行します

#### Parameters

| key         | type   | description                       |
| ----------- | ------ | --------------------------------- |
| target_name | string | MoveGroupの名前付き目標姿勢の名前 |

#### Example

```yaml
ready:
  trigger: [T, 1]
  enabled_states: [arm]
  action:
    - type: moveit_named_target
      target_name: ready
```

### moveit_partial_joint

一部の関節のみの定型動作を実行します

#### Parameters

| key     | type                | description                              |
| ------- | ------------------- | ---------------------------------------- |
| joints  | map(string, float)  | 制御するジョイント名と目標関節角のマッピング |

#### Example

```yaml
up_wrist_joints:
  trigger: [[O, C_U], 0]
  enabled_states: [arm]
  action:
    - type: moveit_partial_joint
      joints:
        arm/joint4: 1.57
        arm/joint5: 0
        arm/joint6: 0
```

### moveit_servo_joint

MoveIt ServoのFKを実行します

#### Parameters

| key                 | type                 | description                                            |
| ------------------- | -------------------- | ------------------------------------------------------ |
| frame_id            | string               | 基準フレーム(FKではあまり関係ない)                     |
| scale               | float                | 速度スケール[rad/s]                                    |
| quiet_on_zero       | bool                 | 出力関節角速度がすべてゼロのときにトピックを配信しない |
| mapping/joint_names | dict(string, string) | 関節名とジョイパッドのボタン名のマッピング             |
| mapping/value       | string or string[]   | 制御量を与えるジョイパッドの軸名，十字キーなどの正負で押すものが変わる際には配列で先頭に正の値となる軸名を，負の値となる軸名を入れる．符号反転が必要な場合は軸軸名の前に-をつける              |

#### Example

```yaml
servo_joint:
  trigger: always
  enabled_states: [arm-fk]
  action:
    - type: moveit_servo_joint
      frame_id: base_link
      scale: 0.1
      mapping:
        joints:
          panda_joint1: X
          panda_joint2: Q
          panda_joint3: T
          panda_joint4: O
          panda_joint5: L1
          panda_joint6: R1
          panda_joint7: OP
        value: [C_U, C_D]
```

### moveit_servo_twist

MoveIt ServoのIKを実行します

#### Parameters

| key               | type   | description                                            |
| ----------------- | ------ | ------------------------------------------------------ |
| frame_id          | string | 基準フレーム                                           |
| scale/translation | float  | 速度スケール[m/s]                                      |
| scale/rotation    | float  | 角速度スケール[rad/s]                                  |
| quiet_on_zero     | bool   | 出力がすべてゼロのときにトピックを配信しない           |
| mapping/x         | string | 基準フレームのX軸並進速度を指定するジョイパッドの軸名  |
| mapping/y         | string | 基準フレームのY軸並進速度を指定するジョイパッドの軸名  |
| mapping/z         | string | 基準フレームのZ軸並進速度を指定するジョイパッドの軸名  |
| mapping/R         | string | 基準フレームのロール角速度を指定するジョイパッドの軸名 |
| mapping/P         | string | 基準フレームのピッチ角速度を指定するジョイパッドの軸名 |
| mapping/Y         | string | 基準フレームのヨー角速度を指定するジョイパッドの軸名   |

#### Example

```yaml
servo_twist:
  trigger: always
  enabled_states: [arm]
  action:
    - type: moveit_servo_twist
      frame_id: panda_link0
      scale:
        translation: 0.05
        rotation: 0.15
      mapping:
        x: LSV
        y: LSH
        z: [R2, -L2]
        R: -RSH
        P: RSV
        Y: [L1, -R1]
```

### publish

任意のトピックにメッセージを配信します

#### Parameters

| key        | type   | description                                                          |
| ---------- | ------ | -------------------------------------------------------------------- |
| topic/name | string | トピック名                                                           |
| topic/type | string | メッセージ型                                                         |
| values     | dict   | メッセージのデータ．yaml形式でトピックの型に対応するデータを構成する |

#### Example

```yaml
hello:
  trigger: always
  action:
    - type: publish
      topic:
        name: hello
        type: std_msgs/String
      values:
        data: Hello
```

### shared_list

配列型の共有変数を操作します

#### Parameters

| key       | type   | description                                                                      |
| --------- | ------ | -------------------------------------------------------------------------------- |
| key       | string | 共有変数マップのキー                                                             |
| value     | any    | 文字列，整数値，少数値のkeyに対応する値．内部でvaluesのサイズ1の配列に変換される |
| values    | list   | keyに対応するリスト                                                              |
| wrap      | bool   | Trueの場合，リストの最後の要素に達したら最初の要素に戻る                         |
| direction | string | 増減方向．"inc"または"increment"で増加                                           |
| initial   | any    | 初期値インデックス値                                                             |

#### Example

```yaml
switch_state_to_common:
  trigger: [OP, OP]
  action:
    - type: shared_list
      key: state
      values: [common, arm]
      direction: inc
      initial: 0
```

### shared_value

スカラー型の共有変数を操作します

#### Parameters

| key           | type   | description                                                            |
| ------------- | ------ | ---------------------------------------------------------------------- |
| key           | string | 共有変数マップのキー                                                   |
| value         | any    | 文字列，整数値，少数値のkeyに対応する値                                |
| step          | float  | 増減量                                                                 |
| enable_button | string | 有効化するボタン名．ボタンが押されている間実行する処理がある場合に使用 |
| initial       | any    | 初期値                                                                 |
| limit/max     | float  | 値の最大値                                                             |
| limit/min     | float  | 値の最小値                                                             |

#### Example

```yaml
sample_inc:
  enabled_states: common
  trigger: always
  action:
    - type: shared_value
      key: float_data
      step: 0.1
      enable_button: C_U
      initial: 0.5
      limit:
        min: -1.
        max: 1.
```


## Math Expressions

| function    | description                                     |
| ----------- | ----------------------------------------------- |
| axis(key)   | keyに対応するジョイパッドの軸の値を取得する     |
| button(key) | keyに対応するジョイパッドのボタンの値を取得する |
| shared(key) | keyが共有変数マップのキーとなる値を取得する     |
