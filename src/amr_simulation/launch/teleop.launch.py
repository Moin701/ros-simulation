"""Keyboard teleoperation for the mecanum-wheeled AMR.

Remaps teleop_twist_keyboard's default 'cmd_vel' output to the topic our
installed mecanum_drive_controller actually listens on. That controller is a
ChainableController (ros-humble-mecanum-drive-controller 2.53.3) configured
with use_stamped_vel: false in controllers.yaml, so it expects plain
geometry_msgs/Twist on <controller_name>/reference_unstamped - not /cmd_vel,
and not TwistStamped. teleop_twist_keyboard's own 'stamped' parameter
defaults to False, so it already publishes plain Twist by default (verified
against the installed teleop_twist_keyboard.py) - the two match with no
further parameters needed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Run directly (no xterm wrapper): ros2 launch does not redirect a child
    # node's stdin, so teleop_twist_keyboard's raw termios keyboard capture
    # works fine reading from the terminal ros2 launch itself was started in.
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/mecanum_drive_controller/reference_unstamped')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Webots) clock'
        ),
        teleop_node,
    ])
