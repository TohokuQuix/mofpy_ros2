from rclpy import Node
from moveit.moveit_py.moveit.core.robot_state import RobotState
from moveit.moveit_py.moveit.planning import MoveItPy, PlanningComponent, TrajectoryExecutionManager

class MoveGroupUtils:
    moveit : MoveItPy = None
    planner : PlanningComponent = None
    executor : TrajectoryExecutionManager = None
    
    def __init__(self):
        pass

    @staticmethod
    def connect(node: Node, planning_group, namespace):
        MoveGroupUtils.moveit = MoveItPy(node_name=node.get_name() + "_moveit", name_space=namespace)
        
        # Sometimes, MoveGroupCommander fails to initialize.
        # Try several times and then give up.
        attempt = 0
        while attempt <= 3:
            try:
                MoveGroupUtils.planner = MoveGroupUtils.moveit.get_planning_component(planning_group)
                MoveGroupUtils.executor = MoveGroupUtils.moveit.get_trajectory_execution_manager()
                return True
            except Exception as e:
                node.get_logger().error(f"Failed to initialize MoveIt planning component: {str(e)}")
        return False