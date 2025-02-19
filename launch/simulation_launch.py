from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions

def generate_launch_description():
    ld = LaunchDescription()
    controller_node = Node(
        package= 'ros2_controller',
        namespace= '',
        executable= 'controlla_node',
        output='screen',
        parameters=[
            {"angular_kp" : 1.73},
            {"angular_ki" : 0.173},
            {"angular_kd" : 0.41},
            {"linear_kp" : 0.015},
            {"linear_ki" : 0.010},
            {"linear_kd" : 0.020},
        ]
    )
    ld.add_action(controller_node)

    return ld
