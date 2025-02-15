import rclpy.logging

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

        if self.has("initial"):
            Shared.add(self.__key, self.get("initial", self.__value))

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

        return current_val + self.__step


Action.register_preset(SharedValue)
