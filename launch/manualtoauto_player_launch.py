import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    manualtoauto_cfg_dir = LaunchConfiguration("manualtoauto_cfg_dir")



    manualtoauto_dir = get_package_share_directory("manualtoauto")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="True", description="Flag to launch RViz."
    )

    declare_manualtoauto_cfg_dir = DeclareLaunchArgument(
        "manualtoauto_cfg_dir",
        default_value=PathJoinSubstitution([manualtoauto_dir, "config", "params.yaml"]),
        description="Path to the manualtoauto config file",
    )


    start_manualtoauto_node = Node(
        package="manualtoauto",
        executable="manualtoauto_player",
        name="manualtoauto_player",
        parameters=[manualtoauto_cfg_dir], 
        output="screen",
    )



    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_rviz)
    ld.add_action(declare_manualtoauto_cfg_dir)
    ld.add_action(start_manualtoauto_node)


    return ld