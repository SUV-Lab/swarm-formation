from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml
import os
from datetime import datetime

def load_yaml_file(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def create_drone_nodes(context, *args, **kwargs):

    record_bag_str = context.perform_substitution(LaunchConfiguration('record_bag'))
    record_bag = (record_bag_str.lower() == 'true')

    # Map name passed in from RViz LaunchControlPanel (or empty when launched
    # straight from a terminal). Empty string falls through to optimizer_params
    # yaml's manager/world default; a non-empty value wins via ROS param order.
    world_arg = context.perform_substitution(LaunchConfiguration('world'))

    disable_file_logging_str = context.perform_substitution(LaunchConfiguration('disable_file_logging'))
    disable_file_logging = (disable_file_logging_str.lower() == 'true')

    # Set environment variable for C++ code (needs "1" not "true")
    if disable_file_logging:
        os.environ['SWARM_DISABLE_FILE_LOGGING'] = '1'
        print("File logging disabled (logs will only appear in console)")
    else:
        os.environ['SWARM_DISABLE_FILE_LOGGING'] = '0'
        print("File logging enabled (logs will be saved to ./logs/runtime)")

    # Target drone ID
    drone_id_str = context.perform_substitution(LaunchConfiguration('drone_id'))
    target_drone_id = int(drone_id_str)
    print(f"Target drone ID: {target_drone_id}")

    # Config paths
    pkg_share = FindPackageShare('path_manager')
    optimizer_file  = PathJoinSubstitution([pkg_share, 'config', 'optimizer_params.yaml'])
    # The `scenario` yaml mechanism is gone (2026-07 structure audit): its last
    # remaining file, scenario_empty.yaml, carried two string params read by
    # nothing, and the static-obstacle pipeline it once fed was deleted.
    # Obstacles are runtime-only now: /dynamic_obstacles/load via the
    # ObstacleScenario panel or a mission yaml.

    # Load base drone hardware configuration
    drones_file = PathJoinSubstitution([pkg_share, 'config', 'drone_hardware.yaml'])
    drones_params = load_yaml_file(context.perform_substitution(drones_file))
    drone_cfg = drones_params['/**']['ros__parameters']
    num_drones = drone_cfg.get('num_drones', 1)
    print(f"Loaded drone hardware config: drone_hardware.yaml (num_drones={num_drones})")
    print("Note: Start positions will be provided via TrajectoryCommand")

    replan_nodes = []

    # Agents to run: all agents configured in drone_hardware.yaml
    # (single-agent setup has just one).
    drones_to_run = []
    for i in range(6):  # Check drone_0 to drone_5
        drone_key = f'drone_{i}'
        if drone_key in drone_cfg:
            drones_to_run.append(drone_cfg[drone_key]['index'])
    print(f"Running agents: {drones_to_run}")

    # Create nodes per drone
    for drone_index in drones_to_run:
        target_cfg = None
        target_key_index = 0

        for i in range(6):  # drone_0 .. drone_5
            drone_key = f'drone_{i}'
            if drone_key in drone_cfg:
                if drone_cfg[drone_key]['index'] == drone_index:
                    target_cfg = drone_cfg[drone_key]
                    target_key_index = i
                    print(f"Found {drone_key} with index={drone_index}")
                    break

        if target_cfg is None:
            print(f"Error: index {drone_index} not found in drones.yaml")
            continue

        idx = drone_index
        i = target_key_index

        params = {
            'drone_id':        idx,
        }
        # Inject manager/world only when the user actually passed one in.
        # Otherwise the yaml default stays in effect.
        if world_arg:
            params['manager/world'] = world_arg
        # Note: start_point will be received from TrajectoryCommand message

        # `params` goes LAST so launch-time overrides (e.g. manager/world from
        # the RViz LaunchControlPanel) win over the yaml defaults.
        replan_params = [optimizer_file, drones_file, params]

        # Node names keep the _drone_{i} suffix so multiple agents stay unique
        # when scaled up; topics are flat for the single agent (see topic_prefix
        # in the C++ sources). The RViz MissionConfig panel matches on the
        # replan_fsm_drone_ prefix, so keep it.
        replan_nodes.append(
            Node(
                package='path_manager',
                executable='path_manager_node',
                name=f'replan_fsm_drone_{i}',
                output='screen',
                parameters=replan_params,
            )
        )

    # Build parameters for the visualization node
    viz_params = [
        drones_file,  # Base drone hardware
        optimizer_file,
    ]

    visualization_node = Node(
        package='mmp_visualization',
        executable='mmp_visualization_node',
        name='mmp_visualization_node',
        output='screen',
        parameters=viz_params,
        condition=IfCondition(LaunchConfiguration('enable_visualization'))
    )

    # NOTE: RViz is now launched separately via:
    #   ros2 launch mmp_visualization mmp.launch.py
    # This allows unified visualization with terrain and all path planning topics

    # Missions come from the RViz MissionConfig panel (/trajectory_command).
    immediate_actions = [visualization_node]

    replan_nodes_delayed = TimerAction(
        period=0.0,
        actions=replan_nodes,
    )

    # ROSbag recording (optional)
    rosbag_actions = []
    if record_bag:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_dir = './logs'
        bag_name = f"drone{target_drone_id}_trajectory_{timestamp}"
        bag_path = os.path.join(bag_dir, bag_name)

        # Create directory if it doesn't exist
        os.makedirs(bag_dir, exist_ok=True)

        record_topics = ['/planning/trajectory', '/planning/initial_trajectory']

        print(f"ROSbag recording enabled: {bag_path}")
        print(f"Recording: {', '.join(record_topics)}")
        rosbag_process = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_path] + record_topics,
            output='screen',
            shell=False
        )
        rosbag_actions = [rosbag_process]
    else:
        print("ROSbag recording disabled")

    print("Launch complete.")
    return immediate_actions + [replan_nodes_delayed] + rosbag_actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drone_id',
            default_value='1',
            description='Target drone ID to run (0-5)'
        ),
        DeclareLaunchArgument(
            'record_bag',
            default_value='false',
            description='Enable rosbag recording for trajectory topics'
        ),
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='false',
            description='Enable path visualization node'
        ),
        DeclareLaunchArgument(
            'disable_file_logging',
            default_value='false',
            description='Disable file logging (logs will only appear in console)'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Map name (e.g. full_map, small_island). When set, '
                        'overrides manager/world in optimizer_params.yaml.'
        ),
        OpaqueFunction(function=create_drone_nodes),
    ])
