import threading
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile

from .action import Action
from ..math_expression import MathExpression
from ..move_group_utils import MoveGroupUtils
from ..shared import Shared


class MoveitServoTwist(Action):

    NAME = "moveit_servo_twist"

    def __init__(self, definition, node: Node):
        super(MoveitServoTwist, self).__init__(definition, node)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__frame_id = self.get("frame_id", "base_link")
        self.__scale_trn = self.get("scale/translation", 0.1)
        self.__scale_rot = self.get("scale/rotation", 0.1)
        self.__quiet_on_zero = self.get("quiet_on_zero", True)
        self.__mapping = self.__mapping__()
        self.__published_zero = False

        self.__pub = node.create_publisher(
            TwistStamped, MoveGroupUtils.servo_node_name + "/delta_twist_cmds", QoSProfile(depth=10)
        )

        client_node = Node(node.get_name() + "_moveit_servo_twist")
        self.__client = client_node.create_client(
            ServoCommandType, MoveGroupUtils.servo_node_name + "/switch_command_type"
        )

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(client_node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def execute(self, named_joy=None):
        if Shared.get("move_group_disabled"):
            return

        if Shared.get("moveit_servo_command_type") != ServoCommandType.Request.TWIST:
            if not self.__servo_init__(self.node):
                rclpy.logging.get_logger("mofpy.MoveitServoTwist").error(
                    "Failed to initialize servo command type"
                )
                return

        scale_trn, scale_rot = self.__get_scales__(named_joy)
        if scale_trn is None or scale_rot is None:
            return

        twist, is_quiet = self.__get_twist__(named_joy["axes"], scale_trn, scale_rot)

        if self.__quiet_on_zero:
            if is_quiet:
                # Publish the all-zero message just once
                if not self.__published_zero:
                    self.__pub.publish(twist)
                    self.__published_zero = True
                return

        self.__pub.publish(twist)
        self.__published_zero = False

    def __servo_init__(self, node: Node):
        while not self.__client.wait_for_service(1):
            rclpy.logging.get_logger("moveit_servo_twist").info(
                "waiting for service {} available...".format(self.__client.service_name)
            )
            pass

        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST
        rclpy.logging.get_logger("moveit_servo_twist").info(
            "call service {}".format(self.__client.service_name)
        )

        self.future = self.__client.call_async(req)
        if not self.__wait_future__(self.future, timeout_sec=1.0):
            rclpy.logging.get_logger("moveit_servo_twist").error(
                "timed out waiting for {}".format(self.__client.service_name)
            )
            return False

        res: ServoCommandType.Response = self.future.result()

        if not res:
            rclpy.logging.get_logger("moveit_servo_twist").error(
                "failed to get response from {}".format(self.__client.service_name)
            )
            return False

        rclpy.logging.get_logger("moveit_servo_twist").info(
            "get response from {}".format(self.__client.service_name)
        )
        if res.success:
            Shared.update("moveit_servo_command_type", ServoCommandType.Request.TWIST)

        return res.success

    def __wait_future__(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.01)
        return future.done()

    def __get_scales__(self, named_joy):
        named_buttons = named_joy["buttons"] if named_joy else {}
        named_axes = named_joy["axes"] if named_joy else {}

        scale_trn = self.__resolve_scale__(self.__scale_trn, named_buttons, named_axes)
        scale_rot = self.__resolve_scale__(self.__scale_rot, named_buttons, named_axes)
        return scale_trn, scale_rot

    def __resolve_scale__(self, value, named_buttons, named_axes):
        if isinstance(value, str):
            resolved, success = MathExpression.expressions(
                {"value": value}, named_buttons=named_buttons, named_axes=named_axes
            )
            if not success:
                rclpy.logging.get_logger("mofpy.MoveitServoTwist").error(
                    "Failed to expand moveit_servo_twist scale expression"
                )
                return None
            value = resolved["value"]

        try:
            return float(value)
        except (TypeError, ValueError):
            rclpy.logging.get_logger("mofpy.MoveitServoTwist").error(
                "moveit_servo_twist scale must resolve to a float"
            )
            return None

    def __get_twist__(self, named_axes, scale_trn, scale_rot):
        dx = scale_trn * self.__get_value__("x", named_axes)
        dy = scale_trn * self.__get_value__("y", named_axes)
        dz = scale_trn * self.__get_value__("z", named_axes)
        d_roll = scale_rot * self.__get_value__("R", named_axes)
        d_pitch = scale_rot * self.__get_value__("P", named_axes)
        d_yaw = scale_rot * self.__get_value__("Y", named_axes)

        twist = Twist()
        twist.linear.x = dx
        twist.linear.y = dy
        twist.linear.z = dz
        twist.angular.x = d_roll
        twist.angular.y = d_pitch
        twist.angular.z = d_yaw

        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.__frame_id
        msg.twist = twist

        is_quiet = all(val == 0 for val in [dx, dy, dz, d_roll, d_pitch, d_yaw])
        return msg, is_quiet

    def __mapping__(self):
        params = self.get("mapping", {})
        mapping = {}
        for key in params.keys():
            val = params[key]
            if type(val) is tuple or type(val) is list:
                mapping[key] = [val[0], val[1]]
            else:
                mapping[key] = [val]

        return mapping

    def __get_value__(self, axis, named_axes):
        """
        Extract the axis/buttons value from joy.

        :param axis: one of x, y, z, R, P, Y to get the value of
        :param named_axes: the processed joy values to get the value from
        :return: the value
        """
        if axis not in self.__mapping:
            return 0

        # List of button names to be added in order to get the value.
        # A name could start with '-', indicating to invert the value
        names = self.__mapping[axis]

        val = 0
        for name in names:
            v = named_axes[name.lstrip("-")].value
            if name.startswith("-"):
                v = -v
            val += v
        return val


Action.register_preset(MoveitServoTwist)
