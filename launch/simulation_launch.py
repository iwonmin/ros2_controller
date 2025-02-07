from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions

def generate_launch_description():
    ld = LaunchDescription()
    controller_node = Node(
        package= 'ros2_controller',
        namespace= '',
        executable= 'controlla_node',
        output='screen'
    )
    ld.add_action(controller_node)

    return ld
