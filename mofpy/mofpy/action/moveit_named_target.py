import rclpy
import rclpy.logging

from .action import Action
from ..move_group_utils import MoveGroupUtils
from ..shared import Shared


class MoveitNamedTarget(Action):
    """
    Control the movement towards a named target pose stored in MoveIt.

    :type __moveit: MoveitPy
    :type __planner: PlanningComponent
    :type __executor: TrajectoryExecutionManager
    """

    NAME = "moveit_named_target"

    def __init__(self, definition, node):
        super(MoveitNamedTarget, self).__init__(definition, node)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__moveit = MoveGroupUtils.moveit
        self.__planner = MoveGroupUtils.planner
        self.__executor = MoveGroupUtils.executor
        self.__target_name = self.get_required("target_name")
        self.__time_from_start = self.get("time_from_start", None)

    def execute(self, named_joy=None):
        if Shared.get("move_group_disabled"):
            msg = "move_group disabled; not executing: {0}".format(self.__target_name)
            rclpy.logging.get_logger("mofpy.MoveitNamedTarget").error(msg)
            return

        self.__move__()

    def __move__(self):
        rclpy.logging.get_logger("mofpy.MoveitNamedTarget").info(
            "Moving to {0}".format(self.__target_name)
        )
        self.__planner.set_start_state_to_current_state()

        if self.__target_name not in self.__planner.named_target_states:
            rclpy.logging.get_logger("mofpy.MoveitNamedTarget").error(
                "could not find the named target '{}'. Please select from the list '{}'".format(
                    self.__target_name, self.__planner.named_target_states
                )
            )
            return

        self.__planner.set_goal_state(configuration_name=self.__target_name)
        plan = self.__planner.plan()

        if plan:
            self.__retime_trajectory(plan.trajectory)
            self.__moveit.execute(plan.trajectory, controllers=[])

    def __retime_trajectory(self, trajectory):
        logger = rclpy.logging.get_logger("mofpy.MoveitNamedTarget")

        if self.__time_from_start is None:
            return

        if (
            not isinstance(self.__time_from_start, (int, float))
            or float(self.__time_from_start) <= 0.0
        ):
            logger.error(
                "Invalid time_from_start '{}'. Must be > 0.0 [s].".format(self.__time_from_start)
            )
            return

        traj_msg = trajectory.get_robot_trajectory_msg()
        points = traj_msg.joint_trajectory.points
        if not points:
            logger.warn("No joint trajectory points to retime")
            return

        current_last = self.__duration_to_sec(points[-1].time_from_start)
        target_last = float(self.__time_from_start)
        if current_last <= 0.0:
            logger.warn("Current trajectory duration is 0; skip retiming")
            return

        ratio = target_last / current_last
        for p in points:
            t = self.__duration_to_sec(p.time_from_start)
            self.__set_duration_from_sec(p.time_from_start, t * ratio)
            # Keep derivatives consistent with retimed timestamps.
            # If time is scaled by ratio, velocity should be 1/ratio and
            # acceleration should be 1/ratio^2.
            if p.velocities:
                p.velocities = [v / ratio for v in p.velocities]
            if p.accelerations:
                p.accelerations = [a / (ratio * ratio) for a in p.accelerations]

        planning_scene_monitor = self.__moveit.get_planning_scene_monitor()
        with planning_scene_monitor.read_only() as scene:
            current_state = scene.current_state
            trajectory.set_robot_trajectory_msg(current_state, traj_msg)

        logger.info(
            "Retimed named target '{}' from {:.3f}s to {:.3f}s (x{:.3f})".format(
                self.__target_name, current_last, target_last, ratio
            )
        )

    @staticmethod
    def __duration_to_sec(duration):
        return float(duration.sec) + float(duration.nanosec) / 1e9

    @staticmethod
    def __set_duration_from_sec(duration, sec_float):
        sec_int = int(sec_float)
        nanosec = int(round((sec_float - sec_int) * 1e9))
        if nanosec >= 1_000_000_000:
            sec_int += 1
            nanosec -= 1_000_000_000
        duration.sec = sec_int
        duration.nanosec = nanosec


Action.register_preset(MoveitNamedTarget)
