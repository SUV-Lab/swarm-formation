import os

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _dynamics_sim(optimizer_params):
    """The trajectory-follower node, or a notice when it is not installed.

    Gated on the EXECUTABLE, not the package. mmp_dynamics_sim is a submodule of
    the MMP workspace, so the package is present after a build — but the version
    on its main branch is a Python 6-DoF sim whose entry points are
    missile_sim_node / waypoint_sequencer / tracking_error_node. It is carried in
    the tree for the upcoming merge with mmp_vehicle_dynamics and is not meant to
    run yet. Checking the package would therefore pass and then die on a missing
    executable, and launch tears the WHOLE tree down when one entity raises — so
    path_manager would not come up either, and the RViz Start button (which forks
    this file) would appear to do nothing, the error buried in the forked
    process's output.
    """
    exe = None
    try:
        prefix = get_package_prefix('mmp_dynamics_sim')
        cand = os.path.join(prefix, 'lib', 'mmp_dynamics_sim', 'dynamics_sim_node')
        exe = cand if os.path.exists(cand) else None
    except PackageNotFoundError:
        pass
    if exe is None:
        return LogInfo(msg='[rviz_path_manager] dynamics_sim_node not installed '
                           '— skipping the trajectory follower. Planning and '
                           'RViz are unaffected; /dynamics/sim_state and '
                           '/dynamics/sim_path stay silent (their RViz displays '
                           'ship disabled).')
    # Integrates the same fixed-wing model used by the optimizer and
    # publishes /dynamics/sim_state + /dynamics/sim_path + vehicle TF.
    return Node(
        package='mmp_dynamics_sim',
        executable='dynamics_sim_node',
        name='dynamics_sim_node',
        output='screen',
        parameters=[optimizer_params],
    )


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

        _dynamics_sim(optimizer_params),
    ])
