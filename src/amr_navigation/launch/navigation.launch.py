"""Full Nav2 autonomy stack (Milestone 3): static map + AMCL localization,
NavFn global planning, DWB local control (holonomic-tuned for the mecanum
AMR), and behavior/BT execution - coordinated by nav2_lifecycle_manager so
every server comes up active automatically. Run alongside
amr_simulation/sim.launch.py, which provides the EKF-fused odom -> base_link
TF this whole stack depends on."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml_path = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_params_file = os.path.join(
        get_package_share_directory('amr_navigation'),
        'config', 'nav2_params.yaml'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {
            'yaml_filename': map_yaml_path,
            'use_sim_time': use_sim_time,
        }],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        ros_arguments=['--log-level', 'WARN'],
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        ros_arguments=['--log-level', 'WARN'],
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel_raw')],
        ros_arguments=['--log-level', 'WARN'],
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel_raw')],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        ros_arguments=['--log-level', 'WARN'],
    )

    # VelocitySmoother sits between controller_server and twist_mux: takes
    # /cmd_vel_raw (DWB's raw, step-changing output), clamps it to the
    # acc/decel budget in nav2_params.yaml's velocity_smoother block, and
    # publishes /cmd_vel_smoothed - which twist_mux.yaml's "navigation"
    # channel now points at instead of /cmd_vel_raw directly. Is a
    # nav2_util::LifecycleNode (confirmed via `strings` on
    # libvelocity_smoother_core.so), so it needs lifecycle_manager bring-up
    # like every other server here - added to node_names below.
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', '/cmd_vel_raw'),
            ('cmd_vel_smoothed', '/cmd_vel_smoothed'),
        ],
    )

    # Classifies /cmd_vel_raw (controller_server's pre-mux output) into a
    # human-readable maneuver state, logged only on state change. Watches
    # Nav2's own output specifically - it will not see teleop or
    # lidar_docker's /cmd_vel_dock commands, since those are separate
    # twist_mux input channels this node isn't subscribed to.
    maneuver_telemetry_node = Node(
        package='amr_navigation',
        executable='maneuver_telemetry_node.py',
        name='maneuver_telemetry_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator',
                'velocity_smoother',
            ],
        }],
    )

    rf2o_params_file = os.path.join(
        get_package_share_directory('amr_navigation'),
        'config', 'rf2o_params.yaml'
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[rf2o_params_file, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # RF2O's first scan callback looks up base_link -> laser_frame via TF
    # (CLaserOdometry2DNode.cpp's setLaserPoseFromTf(), called once on the
    # first scan). If that lookup fails - a real race, since this launch
    # file has no dependency on amr_simulation/sim.launch.py's
    # robot_state_publisher actually being up yet - RF2O does NOT retry or
    # abort: it silently continues with a garbage laser-to-base transform
    # for the rest of the run (verified: no guard in CLaserOdometry2D::init,
    # module_initialized is set unconditionally). A few seconds' delay
    # gives robot_state_publisher's static TF time to propagate first.
    rf2o_node_delayed = TimerAction(period=3.0, actions=[rf2o_node])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/ros/maps/room_map.yaml'),
            description='Full path to the saved map yaml file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the Nav2 params file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Webots) clock'
        ),

        rf2o_node_delayed,
        map_server_node,
        amcl_node,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        velocity_smoother_node,
        maneuver_telemetry_node,
        lifecycle_manager_node,
    ])
