from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=["-d", f"{get_package_share_directory('turtleler')}/share/rviz_turtleler_viewer.rviz"]
        ),
        Node(
            package='turtleler',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        Node(
            package='turtleler',
            executable='turtle_brain',
            name='turtle_brain',
        ),
        # Node(
        #     package='turtleler',
        #     executable='boundary_detector',
        #     name='boundary_detector',
        # ),
        Node(
            package='turtleler',
            executable='turtle_controller',
            name='controller1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
    ])