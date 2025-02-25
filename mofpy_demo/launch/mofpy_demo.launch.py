import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from moveit_py_configs_utils import MoveItPyConfigsBuilder


def generate_launch_description():
    moveit_py_config = (
        MoveItPyConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .joint_limits(file_path="config/hard_joint_limits.yaml")
        .to_moveit_configs()
    )
    mofpy_demo_share_dir = get_package_share_directory("mofpy_demo")

    return LaunchDescription(
        [
            # joy_node の起動
            Node(package="joy", executable="joy_node", name="joy"),
            # mofpy_node の起動
            Node(
                package="mofpy",
                executable="mofpy_node",
                name="mofpy",
                parameters=[
                    {
                        "config": [
                            os.path.join(mofpy_demo_share_dir, "config", "presets", "common.yaml"),
                            os.path.join(mofpy_demo_share_dir, "config", "presets", "arm.yaml"),
                            os.path.join(mofpy_demo_share_dir, "config", "ps4_wired.yaml"),
                            # os.path.join(mofpy_demo_share_dir, "config", "nintendo_switch_pro_controller.yaml"),
                        ],
                        "move_group.planning_group": "panda_arm",
                        "move_group.namespace": "",
                    },
                    moveit_py_config.to_dict(),
                ],
            ),
            # arm.launch.py のインクルード
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mofpy_demo_share_dir, "launch", "arm.launch.py")
                )
            ),
        ]
    )
