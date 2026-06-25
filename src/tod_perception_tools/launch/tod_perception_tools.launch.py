from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("tod_perception_tools")
    default_param_file = os.path.join(
        pkg_share, "config", "tod_perception_tools.params.yaml"
    )

    declare_param_file = DeclareLaunchArgument(
        "param_file",
        default_value=default_param_file,
        description="Path to the YAML file with parameters for tod_perception_tools nodes",
    )

    param_file = LaunchConfiguration("param_file")

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo/rosbag) clock if true'
        )

    tod_dummy_perception_pub_node = Node(
        package="tod_perception_tools",
        executable="tod_dummy_perception_pub",
        name="tod_dummy_perception_pub",
        output="screen",
        parameters=[
            {'use_sim_time': use_sim_time},
            param_file
        ],
    )

    return LaunchDescription([
        declare_param_file,
        use_sim_time_arg,
        tod_dummy_perception_pub_node,
    ])
