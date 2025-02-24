import threading

from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile

from .action import Action
from ..move_group_utils import MoveGroupUtils
from ..shared import Shared


class MoveitServoJoint(Action):

    NAME = "moveit_servo_joint"

    def __init__(self, definition, node: Node):
        super(MoveitServoJoint, self).__init__(definition, node)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__frame_id = self.get("frame_id", "base_link")
        self.__scale = self.get("scale", 0.1)
        self.__quiet_on_zero = self.get("quiet_on_zero", True)
        self.__joint_mapping = self.__mapping__("joints")
        self.__value_mapping = self.__mapping__("value")
        self.__published_zero = False

        self.__pub = node.create_publisher(
            JointJog, MoveGroupUtils.servo_node_name + "/delta_joint_cmds", QoSProfile(depth=10)
        )
        client_node = Node(node.get_name() + "_moveit_servo_joint")
        self.__client = client_node.create_client(
            ServoCommandType, MoveGroupUtils.servo_node_name + "/switch_command_type"
        )

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(client_node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        self.__moveit = MoveGroupUtils.moveit
        robot_model = self.__moveit.get_robot_model()
        group_name = MoveGroupUtils.planner.planning_group_name
        self.__joint_model_names = robot_model.get_joint_model_group(group_name).joint_model_names

    def execute(self, named_joy=None):
        if Shared.get("move_group_disabled"):
            return

        if Shared.get("moveit_servo_command_type") != ServoCommandType.Request.JOINT_JOG:
            if not self.__servo_init__(self.node):
                rclpy.logging.get_logger("mofpy.MoveitServoJoint").error(
                    "Failed to initialize servo command type"
                )

        jog_joints, is_quiet = self.__get_jog_joints__(named_joy["buttons"], named_joy["axes"])

        if self.__quiet_on_zero:
            if is_quiet:
                # Publish the all-zero message just once
                if not self.__published_zero:
                    self.__pub.publish(jog_joints)
                    self.__published_zero = True
                return

        self.__pub.publish(jog_joints)
        self.__published_zero = False

    def __servo_init__(self, node: Node):
        while not self.__client.wait_for_service(1):
            continue

        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.JOINT_JOG
        rclpy.logging.get_logger("moveit_servo_joint_jog").info(
            "call service {}".format(self.__client.service_name)
        )

        self.future = self.__client.call_async(req)
        self.executor.spin_until_future_complete(future=self.future, timeout_sec=1)

        res: ServoCommandType.Response = self.future.result()

        if not res:
            rclpy.logging.get_logger("moveit_servo_joint_jog").error(
                "failed to get response from {}".format(self.__client.service_name)
            )
            return False

        rclpy.logging.get_logger("moveit_servo_joint_jog").info(
            "get response from {}".format(self.__client.service_name)
        )
        if res.success:
            Shared.update("moveit_servo_command_type", ServoCommandType.Request.JOINT_JOG)

        return res.success

    def __get_jog_joints__(self, named_buttons, named_axes):
        msg = JointJog()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.__frame_id
        for joint in self.__joint_mapping:
            if joint in self.__joint_model_names:
                v = self.__get_value__(named_axes)
                vel = self.__scale * self.__get_enable_joint__(joint, named_buttons) * v
                msg.joint_names.append(joint)
                msg.velocities.append(vel)

        is_quiet = all(val == 0 for val in msg.velocities)
        return msg, is_quiet

    def __mapping__(self, key=None):
        mapping_key = "mapping" + f"/{key}" if key else key
        params = self.get(mapping_key, {})
        mapping = {}

        if type(params) is not dict:
            return params

        for key in params.keys():
            val = params[key]
            if type(val) is tuple or type(val) is list:
                mapping[key] = [val[0], val[1]]
            else:
                mapping[key] = val

        return mapping

    def __get_enable_joint__(self, joint_name, named_buttons):
        if joint_name not in self.__joint_mapping:
            return 0

        button = self.__joint_mapping[joint_name]
        return 1 if named_buttons[button].value else 0

    def __get_value__(self, named_axes):
        """
        Extract the axis/buttons value from joy.

        :param axis: one of +, - to get the value of
        :param named_axes: the processed joy values to get the value from
        :return: the value
        """
        # List of button names to be added in order to get the value.
        # A name could start with '-', indicating to invert the value
        names = self.__value_mapping

        if type(names) is not list:
            return named_axes[names].value

        val = 0
        for name in names:
            v = named_axes[name.lstrip("-")].value
            if name.startswith("-"):
                v = -v
            val += v
        return val


Action.register_preset(MoveitServoJoint)
