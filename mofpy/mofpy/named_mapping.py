from .definition import Definitions
from .joy_mapping import VirtAxis
from .joy_mapping import VirtButton


class NamedMappings:
    """
    Maps raw joystick inputs to named virtual axes and buttons.

    This class retrieves predefined virtual input definitions, processes
    joystick messages, and provides a structured dictionary containing
    named axes and buttons.

    :type __v_axes: dict[str, JoyMapping]
    :type __v_buttons: dict[str, JoyMapping]
    """

    def __init__(self):

        self.__v_axes = {}
        v_axes = Definitions.get("virtual/axes")
        for name in v_axes.keys():
            definition = Definitions.get("virtual/axes/{0}".format(name))
            self.__v_axes[name] = VirtAxis(name, definition)

        self.__v_buttons = {}
        v_buttons = Definitions.get("virtual/buttons")
        for name in v_buttons.keys():
            definition = Definitions.get("virtual/buttons/{0}".format(name))
            self.__v_buttons[name] = VirtButton(name, definition)

    def convert(self, msg):
        """
        Process the raw Joy message and converts them into a named, normalized dictionary.

        :param msg:
        :type msg: Joy
        :return:
        """
        new_axes = {}
        new_buttons = {}

        # Axes
        for k in self.__v_axes.keys():
            va = self.__v_axes[k]
            try:
                va.update_value(msg)
            except IndexError:
                msg = "Invalid axis index {0} for {1}".format(va.real_index, va.name)
                raise IndexError(msg)
            new_axes[va.name] = va

        # Buttons
        for k in self.__v_buttons.keys():
            vb = self.__v_buttons[k]
            try:
                vb.update_value(msg)
            except IndexError:
                msg = "Invalid button index {0} for {1}".format(vb.real_index, vb.name)
                raise IndexError(msg)
            new_buttons[vb.name] = vb

        named_joy = {
            "axes": new_axes,
            "buttons": new_buttons,
        }
        return named_joy
