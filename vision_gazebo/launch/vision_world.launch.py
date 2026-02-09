from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():

    world = os.path.expanduser(
        '~/ros2_ws/src/vision_gazebo/worlds/vision_world.world'
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            world
        ],
        output='screen'
    )

    motion = ExecuteProcess(
        cmd=['python3',
             os.path.expanduser('~/ros2_ws/src/vision_gazebo/scripts/motion_sequence.py')],
        output='screen'
    )

    return LaunchDescription([gazebo, motion])

