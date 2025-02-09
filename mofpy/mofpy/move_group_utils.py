from moveit.planning import MoveItPy
from moveit.planning import PlanningComponent
from moveit.planning import TrajectoryExecutionManager
from rclpy.node import Node


class MoveGroupUtils:
    moveit: MoveItPy = None
    planner: PlanningComponent = None
    executor: TrajectoryExecutionManager = None

    def __init__(self):
        pass

    @staticmethod
    def connect(node: Node, planning_group, namespace):
        try:
            MoveGroupUtils.moveit = MoveItPy(
                node_name=node.get_name() + "_moveit", name_space=namespace
            )
        except Exception as e:
            node.get_logger().error(f"Failed to initialize MoveItPy: {str(e)}")
            return False

        # Sometimes, MoveGroupCommander fails to initialize.
        # Try several times and then give up.
        attempt = 0
        while attempt <= 3:
            try:
                MoveGroupUtils.planner = MoveGroupUtils.moveit.get_planning_component(
                    planning_group
                )
                MoveGroupUtils.executor = MoveGroupUtils.moveit.get_trajectory_execution_manager()
                return True
            except Exception as e:
                node.get_logger().error(f"Failed to initialize MoveIt planning component: {str(e)}")
                attempt += 1
        return False
