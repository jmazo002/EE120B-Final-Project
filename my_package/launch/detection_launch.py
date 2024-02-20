from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='detection',
            name='DETECTION'
        ),
        Node(
            package='my_package',
            executable='display',
            name='DISPLAY'
        ),
        Node(
            package='my_package',
            executable='controller',
            name='CONTROLLER'
        ),

    ])