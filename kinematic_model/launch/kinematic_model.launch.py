from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()


    kinematic_model_node = Node(
        package='kinematic_model',
        executable='kinematic_model',
        name='kinematic_model_name',
        output='screen'
    )

    wheel_velocities_publisher_node = Node(
        package='wheel_velocities_publisher',
        executable='wheel_velocities_publisher',
        name='wheel_velocities_publisher_name',
        output='screen'
    )

    ld.add_action(kinematic_model_node)
    ld.add_action(wheel_velocities_publisher_node)
    
    return ld