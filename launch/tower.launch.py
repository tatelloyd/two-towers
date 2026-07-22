"""Launch one complete tower: perception node plus behavior tree tracker.

A tower is a camera and a pan/tilt turret that track together. This file brings
up exactly one, inside its own namespace, so that two towers can coexist either
on one Raspberry Pi or across two without any change to the node source.

Usage:
    ros2 launch two_towers tower.launch.py
    ros2 launch two_towers tower.launch.py tower_id:=tower_b
    ros2 launch two_towers tower.launch.py tower_id:=tower_b enable_streaming:=false

The namespace is what makes this work. Both nodes use relative topic names
('detections'), so pushing them into /tower_a produces /tower_a/detections and
pushing them into /tower_b produces /tower_b/detections, with no tower-specific
strings compiled into either binary.
"""

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    tower_id = LaunchConfiguration('tower_id')
    params_file = LaunchConfiguration('params_file')
    enable_streaming = LaunchConfiguration('enable_streaming')
    launch_detector = LaunchConfiguration('launch_detector')
    launch_tracker = LaunchConfiguration('launch_tracker')

    declared_args = [
        DeclareLaunchArgument(
            'tower_id',
            default_value='tower_a',
            description='Namespace for this tower. Determines topic names.',
        ),
        DeclareLaunchArgument(
            'params_file',
            # Defaults to the config file matching tower_id, so
            # tower_id:=tower_b picks up config/tower_b.yaml automatically.
            default_value=PathJoinSubstitution([
                FindPackageShare('two_towers'),
                'config',
                [tower_id, '.yaml'],
            ]),
            description='Parameter file for this tower.',
        ),
        DeclareLaunchArgument(
            'enable_streaming',
            default_value='true',
            description='Run the Flask MJPEG debug stream alongside detection.',
        ),
        # These two exist so a single tower can be split across machines later:
        # run the detector on the Pi with the camera, the tracker on the Pi with
        # the servos, without needing a second launch file.
        DeclareLaunchArgument(
            'launch_detector',
            default_value='true',
            description='Start the YOLO perception node.',
        ),
        DeclareLaunchArgument(
            'launch_tracker',
            default_value='true',
            description='Start the behavior tree tracker node.',
        ),
    ]

    detector = Node(
        package='two_towers',
        executable='two_towers_detector_node.py',
        name='two_towers_detector',
        parameters=[params_file, {'enable_streaming': enable_streaming}],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(launch_detector),
    )

    tracker = Node(
        package='two_towers',
        executable='orthanc_tracker_bt_node',
        name='orthanc_tracker_bt',
        parameters=[params_file],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(launch_tracker),
    )

    tower = GroupAction([
        PushRosNamespace(tower_id),
        detector,
        tracker,
    ])

    return LaunchDescription(declared_args + [tower])
