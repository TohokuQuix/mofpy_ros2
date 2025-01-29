from threading import Thread, Lock
import rclpy
import rclpy.logging
from rclpy.node import Node

from .action.action import Action
from .event_manager import EventManager
from .named_mapping import NamedMappings
from .shared import Shared
from .definition import Definitions

import six

class PresetHandler:
    """
    :type __joy_namer: NamedMappings
    :type __presets: dict[str, list[Action]]
    """
    def __init__(self, joy_names, node: Node) -> None:
        self.__joy_namer = joy_names
        self.__node = node
        self.__named_joy = None
        self.__named_joy_lock = Lock()
        self.__presets = dict()
        self.__enabled_states = dict()
        self.__timer = self.__node.create_timer(1.0 / Definitions.get("rate", 100.), self.__timer_callback__)
        timeout_press = Definitions.get("timeout/press", 0.1)
        timeout_sequence = Definitions.get("timeout/sequence", 0.2)
        self.__event_manager = EventManager(node, timeout_press, timeout_sequence)

        # MoveIt! paramters
        try:
            from .move_group_utils import MoveGroupUtils
            move_group_enabled = self.__node.declare_parameter("move_group.enabled", True)
        except ImportError as e:
            move_group_enabled = False

        if move_group_enabled:
            p = "move_group"
            action_namespace = self.__node.declare_parameter(p + ".action_namespace", "")
            planning_group = self.__node.declare_parameter(p + ".planning_group", "")
            robot_description = self.__node.declare_parameter(p + ".robot_description", "robot_description")
            connected = MoveGroupUtils.connect(planning_group, robot_description, action_namespace)
            move_group_enabled = move_group_enabled and connected
        Shared.add('move_group_disabled', not move_group_enabled)
        if move_group_enabled:
            self.__node.get_logger().info("Connected to MoveIt!")

    def bind_presets(self, definition):
        """
        Binds the triggers (input commands) to the preset to be executed
        :param definition:
        :return:
        """
        for preset_name in definition.keys():
            trigger, es, preset = self.__read_preset__(preset_name)
            self.__presets[preset_name] = preset

            trigger = definition[preset_name]['trigger']
            self.__event_manager.register(preset_name, trigger)

            if isinstance(es, six.string_types):
                self.__enabled_states[preset_name] = [es]
            else:
                self.__enabled_states[preset_name] = es

        # Output error messages about failed import attempts
        for module_name in Action.disabled.keys():
            reason = Action.disabled[module_name]
            msg = 'Failed to import {0} : {1}'.format(module_name, reason)
            self.__node.get_logger().error(msg)

    def handle(self, msg):
        """
        Handles a Joy message
        :param msg: Joy message
        :type msg: Joy
        """
        named_joy = self.__joy_namer.convert(msg)
        self.__event_manager.append(named_joy['buttons'])

        self.__named_joy_lock.acquire(True)
        self.__named_joy = named_joy
        self.__named_joy_lock.release()

    def __timer_callback__(self):
        self.__named_joy_lock.acquire(True)

        triggered_presets = self.__event_manager.get_sequence_triggered()

        # For debugging purposes
        et = self.__enabled_triggered__(triggered_presets)
        if len(et) != 0:
            print(et)

        # Presets that are triggered by commands
        for preset_name in triggered_presets:
            if not self.__is_enabled_state__(preset_name):
                continue

            # Execute each action
            for preset_action in self.__presets[preset_name]:
                preset_action.execute()

        # Presets that are always triggered
        for preset_name in self.__event_manager.get_always_triggered():
            if not self.__is_enabled_state__(preset_name):
                continue
            for preset_action in self.__presets[preset_name]:
                if self.__named_joy is not None:
                    preset_action.execute(self.__named_joy)

        self.__named_joy = None
        self.__named_joy_lock.release()


    def __is_enabled_state__(self, preset_name):
        enabled_states = self.__enabled_states[preset_name]

        # Per-state enabling is turned off
        if not Shared.has('state') or len(enabled_states) == 0:
            return True

        current_state = Shared.get('state')
        return current_state in enabled_states

    def __enabled_triggered__(self, triggered):
        return list(filter(self.__is_enabled_state__, triggered))

    def __read_preset__(self, preset_name):
        actions = []
        preset = Definitions.get("presets/{0}".format(preset_name))
        # preset = self.__node.get_parameters_by_prefix('presets.{0}'.format(preset_name))
        trigger = preset['trigger']
        if 'enabled_states' in preset:
            enabled_states = preset['enabled_states']
        else:
            # Per-state enabling turned off
            enabled_states = []

        for action_definition in preset['action']:
            if 'type' not in action_definition:
                self.__node.get_logger().error('Required key "type" not set')
                continue
            action_type = action_definition['type']
            # Note: Failed imports will be skipped here
            if action_type in Action.actions:
                cls = Action.actions[action_type]
                actions.append(cls(action_definition, self.__node))
            else:
                self.__node.get_logger().warn('action_type "{0}" is not supported : supported list -> {1}'.format(action_type, str(list(Action.actions.keys()))))

        return trigger, enabled_states, actions
