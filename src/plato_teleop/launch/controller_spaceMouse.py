from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("plato_manipulator", package_name="plato_moveit_config").to_moveit_configs()

    tutorial_node = Node(
        package="plato_teleop",
        executable="controller_spaceMouse",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([tutorial_node])