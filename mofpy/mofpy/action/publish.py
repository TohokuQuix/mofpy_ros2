import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.utilities import get_message
import yaml

from .action import Action


class Publish(Action):
    """
    Publish message.

    :type __topic_name: str
    :type __topic_type: str
    :type __values: dict
    :type __latch: bool
    """

    NAME = "publish"

    def __init__(self, definition, node: Node):
        super(Publish, self).__init__(definition, node)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__topic_name = self.get_required("topic/name")
        self.__topic_type = self.get_required("topic/type")
        # TODO: QoSをパラメータで設定可能にする
        self.__qos_profile: QoSProfile = QoSProfile(depth=10)
        self.__values = self.get_required("values")

        self.__pub, self.__msg_class = self.create_publisher(
            self.__topic_name, self.__topic_type, self.__qos_profile
        )

    def create_publisher(self, topic_name, topic_type, qos_profile):
        try:
            msg_class = get_message(topic_type)
        except (AttributeError, ModuleNotFoundError, ValueError):
            raise RuntimeError("The passed message type is invalid")

        pub = self.node.create_publisher(msg_class, topic_name, qos_profile)
        return pub, msg_class

    def execute(self, named_joy=None):
        yaml_vals = yaml.load(str(self.__values), Loader=yaml.FullLoader)
        msg = self.__msg_class()
        try:
            timestamp_fields = set_message_fields(
                msg, yaml_vals, expand_header_auto=True, expand_time_now=True
            )
        except Exception as e:
            rclpy.logging.get_logger("mofpy.Publish").error(
                "Failed to populate field: {0}".format(e)
            )
            return

        now = self.node.get_clock().now().to_msg()
        for field_setter in timestamp_fields:
            field_setter(now)

        self.__pub.publish(msg)


Action.register_preset(Publish)
