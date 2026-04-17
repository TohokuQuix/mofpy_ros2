import math
import time
from typing import Dict

from sensor_msgs.msg import JointState

from .action import Action
from ..shared import Shared


class FlipperTargetUpdate(Action):
    NAME = "flipper_target_update"

    def __init__(self, definition, node):
        super(FlipperTargetUpdate, self).__init__(definition, node)

        self.__joint_state_topic = self.get("joint_state_topic", "/joint_states")
        self.__step = float(self.get("step", 0.05))
        self.__reset_threshold = float(self.get("reset_threshold", math.radians(10.0)))
        self.__idle_reset_timeout = float(self.get("idle_reset_timeout", 0.3))
        self.__control_mode_key = self.get("control_mode_key", "flipper_control_mode")
        self.__direction_key = self.get("direction_key", "chassis_direction")

        self.__shared_keys = self.get(
            "shared_keys",
            {
                "fl": "fl_flipper_target",
                "fr": "fr_flipper_target",
                "rl": "rl_flipper_target",
                "rr": "rr_flipper_target",
            },
        )
        self.__joint_names = self.get(
            "joint_names",
            {
                "fl": "fl_flipper_joint",
                "fr": "fr_flipper_joint",
                "rl": "rl_flipper_joint",
                "rr": "rr_flipper_joint",
            },
        )

        mapping = self.get("mapping", {})
        self.__mapping = {
            "synchronous": {
                "front": {
                    "raise": mapping.get("synchronous", {}).get("front", {}).get("raise", "R1"),
                    "lower": mapping.get("synchronous", {}).get("front", {}).get("lower", "R2"),
                },
                "rear": {
                    "raise": mapping.get("synchronous", {}).get("rear", {}).get("raise", "L1"),
                    "lower": mapping.get("synchronous", {}).get("rear", {}).get("lower", "L2"),
                },
            },
            "independent": {
                "raise": mapping.get("independent", {}).get("raise", "C_U"),
                "lower": mapping.get("independent", {}).get("lower", "C_D"),
                "joints": mapping.get(
                    "independent",
                    {},
                ).get(
                    "joints",
                    {
                        "fl": "L1",
                        "fr": "R1",
                        "rl": "L2",
                        "rr": "R2",
                    },
                ),
            },
        }

        self.__current_positions: Dict[str, float] = {}
        self.__last_targets: Dict[str, float] = {}
        self.__manual_targets: Dict[str, bool] = {key: False for key in self.__shared_keys.keys()}
        self.__warned_missing_joint_state = False
        self.__last_input_time = time.monotonic()

        self.__joint_state_sub = self.node.create_subscription(
            JointState, self.__joint_state_topic, self.__joint_state_cb, 1
        )

    def execute(self, named_joy=None):
        if not self.__has_joint_states():
            if not self.__warned_missing_joint_state:
                self.node.get_logger().warning(
                    "Waiting for flipper joint states before updating shared targets"
                )
                self.__warned_missing_joint_state = True
            return

        self.__warned_missing_joint_state = False
        self.__initialize_targets_from_joint_state()
        self.__sync_external_targets()

        updates = self.__compute_updates(named_joy)
        if updates:
            self.__apply_updates(updates)
            return

        self.__reset_detached_manual_targets()

    def __joint_state_cb(self, msg: JointState):
        name_to_index = {name: idx for idx, name in enumerate(msg.name)}
        for key, joint_name in self.__joint_names.items():
            if joint_name not in name_to_index:
                continue
            index = name_to_index[joint_name]
            if index < len(msg.position):
                self.__current_positions[key] = float(msg.position[index])

    def __has_joint_states(self) -> bool:
        return all(key in self.__current_positions for key in self.__joint_names.keys())

    def __initialize_targets_from_joint_state(self):
        for key, shared_key in self.__shared_keys.items():
            if Shared.has(shared_key):
                if key not in self.__last_targets:
                    self.__last_targets[key] = float(Shared.get(shared_key))
                continue
            current = self.__current_positions[key]
            Shared.add(shared_key, current)
            self.__last_targets[key] = current

    def __sync_external_targets(self):
        for key, shared_key in self.__shared_keys.items():
            target = float(Shared.get(shared_key, self.__current_positions[key]))
            if key not in self.__last_targets:
                self.__last_targets[key] = target
                continue
            if not math.isclose(target, self.__last_targets[key], abs_tol=1.0e-9):
                self.__manual_targets[key] = False
                self.__last_targets[key] = target

    def __compute_updates(self, named_joy=None):
        if named_joy is None:
            return {}

        buttons = named_joy["buttons"]
        mode = str(Shared.get(self.__control_mode_key, "synchronous"))
        direction = -1 if float(Shared.get(self.__direction_key, 1)) < 0 else 1
        updates = {key: 0.0 for key in self.__shared_keys.keys()}

        if mode == "independent":
            delta_sign = self.__button_direction(
                buttons,
                self.__mapping["independent"]["raise"],
                self.__mapping["independent"]["lower"],
            )
            if delta_sign == 0:
                return {}

            joint_buttons = dict(self.__mapping["independent"]["joints"])
            if direction < 0:
                joint_buttons = {
                    "fl": joint_buttons["rr"],
                    "fr": joint_buttons["rl"],
                    "rl": joint_buttons["fr"],
                    "rr": joint_buttons["fl"],
                }

            for key, button_name in joint_buttons.items():
                if self.__button_pressed(buttons, button_name):
                    updates[key] = delta_sign * self.__step

            return {key: value for key, value in updates.items() if value != 0.0}

        front_delta = self.__button_direction(
            buttons,
            self.__mapping["synchronous"]["front"]["raise"],
            self.__mapping["synchronous"]["front"]["lower"],
        )
        rear_delta = self.__button_direction(
            buttons,
            self.__mapping["synchronous"]["rear"]["raise"],
            self.__mapping["synchronous"]["rear"]["lower"],
        )

        front_keys = ("fl", "fr") if direction > 0 else ("rl", "rr")
        rear_keys = ("rl", "rr") if direction > 0 else ("fl", "fr")

        for key in front_keys:
            updates[key] = front_delta * self.__step
        for key in rear_keys:
            updates[key] = rear_delta * self.__step

        return {key: value for key, value in updates.items() if value != 0.0}

    def __apply_updates(self, updates):
        self.__last_input_time = time.monotonic()
        for key, delta in updates.items():
            shared_key = self.__shared_keys[key]
            target = float(Shared.get(shared_key, self.__current_positions[key]))
            target += delta
            Shared.update(shared_key, target)
            self.__last_targets[key] = target
            self.__manual_targets[key] = True

    def __reset_detached_manual_targets(self):
        if time.monotonic() - self.__last_input_time < self.__idle_reset_timeout:
            return
        for key, is_manual in self.__manual_targets.items():
            if not is_manual:
                continue
            shared_key = self.__shared_keys[key]
            target = float(Shared.get(shared_key, self.__current_positions[key]))
            current = self.__current_positions[key]
            if abs(self.__normalize_angle(target - current)) <= self.__reset_threshold:
                continue
            Shared.update(shared_key, current)
            self.__last_targets[key] = current
            self.__manual_targets[key] = False

    @staticmethod
    def __button_pressed(buttons, name):
        return bool(name in buttons and buttons[name].value)

    def __button_direction(self, buttons, raise_button, lower_button):
        return int(self.__button_pressed(buttons, raise_button)) - int(
            self.__button_pressed(buttons, lower_button)
        )

    @staticmethod
    def __normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


Action.register_preset(FlipperTargetUpdate)
