"""Launch Webots with the AMR robot driver, controllers, state publishers, and twist_mux."""

import os
import tempfile

import launch
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir_sim = get_package_share_directory('amr_simulation')
    package_dir_desc = get_package_share_directory('amr_description')

    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config_path = os.path.join(package_dir_desc, 'rviz', 'viewport_config.rviz')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir_sim, 'webots', 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )

    # Flatten the modular xacro to a real URDF file on disk: WebotsController's
    # 'robot_description' parameter expects a file path, not inline XML
    # (confirmed against the installed webots_ros2_turtlebot reference launch file).
    xacro_path = os.path.join(package_dir_desc, 'urdf', 'robots', 'amr.urdf.xacro')
    robot_description_content = xacro.process_file(
        xacro_path, mappings={'use_webots': 'true'}
    ).toxml()
    robot_description_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='.urdf', delete=False)
    robot_description_file.write(robot_description_content)
    robot_description_file.close()
    robot_description_path = robot_description_file.name

    # Publishes TF using our own verified flattened URDF (already validated by
    # check_urdf). set_robot_state_publisher=True on WebotsController is
    # deliberately NOT used: it makes Webots synthesize its own URDF from the
    # live PROTO via wb_robot_get_urdf() and overwrite this node's
    # robot_description with that synthesized copy - which reproducibly
    # generates a duplicate 'base_link' for this robot's PROTO structure
    # ("Error: link 'base_link' is not unique"). Publishing our own correct
    # URDF here and leaving set_robot_state_publisher off avoids that path
    # entirely.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }],
    )

    ros2_control_params = os.path.join(package_dir_desc, 'config', 'amr', 'controllers.yaml')

    amr_driver = WebotsController(
        robot_name='amr_robot',
        parameters=[
            {
                'robot_description': robot_description_path,
                'use_sim_time': use_sim_time,
                'set_robot_state_publisher': False,
            },
            ros2_control_params
        ],
        respawn=True
    )

    controller_manager_timeout = ['--controller-manager-timeout', '50']
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diff_drive_controller'] + controller_manager_timeout,
    )
    ros_control_spawners = [joint_state_broadcaster_spawner, diff_drive_controller_spawner]

    # Wait for Webots to actually connect the driver before spawning controllers
    waiting_nodes = WaitForControllerConnection(
        target_driver=amr_driver,
        nodes_to_start=ros_control_spawners
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # EKF sensor fusion: publishes the odom -> base_link TF that
    # mecanum_drive_controller can't (that controller build has no
    # TransformBroadcaster of its own - see controllers.yaml).
    ekf_params_file = os.path.join(
        package_dir_sim, 'config', 'ekf.yaml'
    )
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}]
    )

    # twist_mux: merges velocity commands from three channels (see
    # amr_navigation/config/twist_mux.yaml for the current priorities -
    #   teleop=100, docking=50, navigation=10).
    # Output is remapped to diff_drive_controller's real command topic:
    # <controller_name>/cmd_vel_unstamped (confirmed via `strings` on
    # libdiff_drive_controller.so - this build, unlike the old
    # mecanum_drive_controller, is the classic topic-subscribing controller,
    # not a ChainableController with a "reference"-style input port; taking
    # cmd_vel_unstamped rather than cmd_vel since use_stamped_vel is false
    # in controllers.yaml, matching twist_mux's plain geometry_msgs/Twist
    # output).
    twist_mux_params = os.path.join(
        get_package_share_directory('amr_navigation'), 'config', 'twist_mux.yaml'
    )
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')],
    )

    # Lidar-reactive docking controller — publishes to /cmd_vel_dock
    lidar_docker_node = Node(
        package='amr_navigation',
        executable='lidar_docker.py',
        name='lidar_docker',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='simulation.wbt',
            description='Webots world file, relative to webots/worlds/'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Webots) clock'
        ),

        webots,
        webots._supervisor,

        robot_state_publisher,
        amr_driver,
        waiting_nodes,
        rviz,
        twist_mux_node,
        lidar_docker_node,
        # ekf_node disabled - RF2O laser odometry now publishes odom -> base_link TF directly

        # Kill all nodes once the Webots simulation exits
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
