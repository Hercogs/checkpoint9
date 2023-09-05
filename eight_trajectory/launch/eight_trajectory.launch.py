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

    eight_trajectory_node = Node(
        package='eight_trajectory',
        executable='eight_trajectory',
        name='eight_trajectory_name',
        output='screen'
    )

    ld.add_action(kinematic_model_node)
    ld.add_action(eight_trajectory_node)
    
    return ld