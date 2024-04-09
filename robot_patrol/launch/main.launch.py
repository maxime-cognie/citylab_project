from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('robot_patrol'),
                '/launch/',
                'start_direction_service.launch.py'
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('robot_patrol'),
                '/launch/',
                'start_patrolling_client.launch.py'
            ]),
        )
    ])