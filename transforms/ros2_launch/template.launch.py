from launch import LaunchDescription
from launch_ros.actions import Node as RosNode

def generate_launch_description():
    ld = LaunchDescription()

    # Robot state publisher
    ld.add_action(RosNode(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='{{config:robot_name}}_state_publisher',
    ))

    # Joint state broadcaster
    ld.add_action(RosNode(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    ))

{{#each revolute_joints}}
    # Controller for {{name}}
    ld.add_action(RosNode(
        package='controller_manager',
        executable='spawner',
        arguments=['{{name}}_controller'],
    ))
{{/each}}

{{#each continuous_joints}}
    # Controller for {{name}}
    ld.add_action(RosNode(
        package='controller_manager',
        executable='spawner',
        arguments=['{{name}}_controller'],
    ))
{{/each}}

    return ld
