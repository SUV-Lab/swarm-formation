from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    RViz simulation launch file for path_manager
    Automatically sets: enable_visualization=true

    Usage:
        ros2 launch path_manager rviz_path_manager.launch.py
    """

    pkg_share = FindPackageShare('path_manager')
    path_manager_launch = PathJoinSubstitution([pkg_share, 'launch', 'path_manager.launch.py'])
    optimizer_params = PathJoinSubstitution([pkg_share, 'config', 'optimizer_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario',
            # Empty -> path_manager.launch.py falls back to scenario_empty
            # (no obstacles). The old scenario_basic default planted a legacy
            # test obstacle at the origin in every RViz-launched run.
            default_value='',
            description='Scenario configuration file (default: scenario_empty = none)'
        ),
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
            'disable_file_logging',
            default_value='false',
            description='Disable file logging (logs will only appear in console)'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Terrain map name. Leave empty to fall back to '
                        'optimizer_params.yaml manager/world.'
        ),
        # Include base path_manager launch with RViz defaults
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path_manager_launch),
            launch_arguments={
                'enable_visualization': 'true',
                'scenario': LaunchConfiguration('scenario'),
                'drone_id': LaunchConfiguration('drone_id'),
                'record_bag': LaunchConfiguration('record_bag'),
                'disable_file_logging': LaunchConfiguration('disable_file_logging'),
                'world': LaunchConfiguration('world'),
            }.items()
        ),

        # Integrates the same fixed-wing model used by the optimizer and
        # publishes /dynamics/sim_state + /dynamics/sim_path + vehicle TF.
        Node(
            package='mmp_dynamics_sim',
            executable='dynamics_sim_node',
            name='dynamics_sim_node',
            output='screen',
            parameters=[optimizer_params],
        ),
    ])
