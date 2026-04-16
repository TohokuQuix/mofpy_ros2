import rclpy.logging
from sensor_msgs.msg import JointState

from .action import Action
from ..shared import Shared


class SharedValue(Action):
    NAME = "shared_value"

    def __init__(self, definition, node):
        super(SharedValue, self).__init__(definition, node)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__key = self.get_required("key")

        self.__value = self.get("value")

        self.__step = self.get("step", 0)

        self.__enable_button = self.get("enable_button")

        self.__has_limit = self.has("limit/max") and self.has("limit/min")
        self.__max_limit = self.get("limit/max")
        self.__min_limit = self.get("limit/min")
        self.__joint_state_sub = None
        self.__joint_state_initialized = False

        if self.has("initial"):
            Shared.add(self.__key, self.get("initial", self.__value))

        if self.has("initial_from_joint_state/joint"):
            topic = self.get("initial_from_joint_state/topic", "/joint_states")
            self.__joint_name = self.get("initial_from_joint_state/joint")
            self.__joint_scale = self.get("initial_from_joint_state/scale", 1.0)
            self.__joint_offset = self.get("initial_from_joint_state/offset", 0.0)
            self.__joint_state_sub = self.node.create_subscription(
                JointState, topic, self.__initialize_from_joint_state, 1
            )

    def execute(self, named_joy=None):
        if not named_joy or not self.__enable_button:
            Shared.update(self.__key, self.__value)
            rclpy.logging.get_logger("shared_value").info(
                "{0} : {1}".format(self.__key, self.__value)
            )
            return

        named_buttons = named_joy["buttons"]
        active_button = (
            named_buttons[self.__enable_button].value
            if self.__enable_button in named_joy["buttons"]
            else None
        )
        if active_button:
            value = self.__next_value__()
            Shared.update(self.__key, value)

            rclpy.logging.get_logger("shared_value").info("{0} : {1}".format(self.__key, value))

    def __next_value__(self):
        if self.__value:
            return self.__value

        current_val = Shared.get(self.__key)

        next_val = current_val + self.__step

        return (
            max(self.__min_limit, min(next_val, self.__max_limit)) if self.__has_limit else next_val
        )

    def __initialize_from_joint_state(self, msg: JointState):
        if self.__joint_state_initialized:
            return

        if self.__joint_name not in msg.name:
            return

        index = msg.name.index(self.__joint_name)
        value = msg.position[index] * self.__joint_scale + self.__joint_offset
        if self.__has_limit:
            value = max(self.__min_limit, min(value, self.__max_limit))

        if Shared.has(self.__key):
            Shared.update(self.__key, value)
        else:
            Shared.add(self.__key, value)

        self.__joint_state_initialized = True
        rclpy.logging.get_logger("shared_value").info(
            'Initialized "{0}" from joint "{1}" = {2}'.format(self.__key, self.__joint_name, value)
        )
        if self.__joint_state_sub is not None:
            self.node.destroy_subscription(self.__joint_state_sub)
            self.__joint_state_sub = None


Action.register_preset(SharedValue)
