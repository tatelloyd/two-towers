"""Launch both towers.

Includes tower.launch.py twice with different namespaces. Everything that makes
a tower distinct -- GPIO pins, camera calibration, stream port -- lives in that
tower's config file, so this file stays a thin composition layer.

Usage:
    ros2 launch two_towers two_towers.launch.py

Both towers on one Pi is the intended bringup path and it works, but note that
two YOLO inference streams will not hold 15 Hz on a single Pi 4. Expect a lower
effective detection rate until the towers are split across two machines.

To run the towers on separate machines, do not use this file. Run
tower.launch.py on each Pi with the matching tower_id, and make sure both
machines share a ROS_DOMAIN_ID and can reach each other.
"""

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    enable_streaming = LaunchConfiguration('enable_streaming')

    tower_launch = PathJoinSubstitution([
        FindPackageShare('two_towers'),
        'launch',
        'tower.launch.py',
    ])

    def tower(tower_id):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tower_launch),
            launch_arguments={
                'tower_id': tower_id,
                'enable_streaming': enable_streaming,
            }.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_streaming',
            default_value='true',
            description=(
                'Run the Flask debug stream for both towers. Worth disabling on '
                'a single Pi: annotation runs inside the detection callback and '
                'competes with inference for CPU.'
            ),
        ),
        tower('tower_a'),
        tower('tower_b'),
    ])
