import rclpy
import rclpy.logging

from moveit.moveit_py.moveit.core.robot_state import RobotState

from .action import Action
from ..move_group_utils import MoveGroupUtils
from ..shared import Shared

class MoveitNamedTarget(Action):
    """
    :type __robot: RobotCommander
    :type __group: MoveGroupCommander
    """

    NAME = 'moveit_named_target'

    def __init__(self, definition, node):
        super(MoveitNamedTarget, self).__init__(definition, node)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__moveit = MoveGroupUtils.moveit
        self.__planner = MoveGroupUtils.planner
        self.__executor = MoveGroupUtils.executor
        self.__target_name = self.get_required('target_name')

    def execute(self, named_joy=None):
        if Shared.get('move_group_disabled'):
            msg = 'move_group disabled; not executing: {0} {1}'.format(
                self.__action,
                self.__target_name
            )
            rclpy.logging.get_logger("mofpy.MoveitNamedTarget").error(msg)
            return

        self.__move__()

    def __move__(self):
        rclpy.logging.get_logger("mofpy.MoveitNamedTarget").info('Moving to {0}'.format(self.__target_name))
        self.__planner.set_start_state_to_current_state()
        named_target_values : dict = self.__planner.get_named_target_state_values()
        
        if not self.__target_name in named_target_values:
            rclpy.logging.get_logger("mofpy.MoveitNamedTarget").error("could not find the named target '{}'. Please select from the list '{}'".format(self.__target_name, named_target_values.values()))
            return
        
        goal_state = RobotState()
        goal_state.robot_model = self.__moveit.get_robot_model()
        goal_state.joint_positions = named_target_values[self.__target_name]
        
        self.__planner.set_goal_state(goal_state)
        plan = self.__planner.plan()
        
        if plan:
            self.__executor.execute(plan.trajectory, controllers=[])


Action.register_preset(MoveitNamedTarget)