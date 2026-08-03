from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import LogInfo
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _follower(context, *args, **kwargs):
    """Explicit follower slot, chosen by the `follower` launch argument.

    This used to probe the filesystem for a dynamics_sim_node executable —
    honest while three follower implementations coexisted, but a probe hides
    intent: nothing said WHICH follower a run wanted. Now the run says it.

    - shared_3dof (default): mmp_vehicle_dynamics' C++ follower — same model
      as the optimizer's cost term. Publishes /dynamics/sim_state + sim_path
      and the drone_0_base / drone_0_chase TFs the follow camera targets.
    - none: no follower; /dynamics stays silent.
    - vehicle_sim: RESERVED for the external vehicle-dynamics follower. Its
      previous incarnation (the mmp_dynamics_sim submodule) was removed from
      the workspace 2026-08 pending a restructure that consumes
      mmp_vehicle_dynamics' Python bindings instead of carrying its own
      physics; the slot stays so the restructured package plugs back in.

    Launch tears the whole tree down when one entity raises, so an invalid
    value logs-and-skips instead of raising: the RViz Start button forks this
    file, and a dead launch tree looks like "Start does nothing".
    """
    choice = context.perform_substitution(LaunchConfiguration('follower'))
    if choice == 'none':
        return [LogInfo(msg='[rviz_path_manager] follower:=none — no dynamics '
                            'follower started.')]
    if choice == 'shared_3dof':
        pkg_share = FindPackageShare('path_manager')
        optimizer_params = PathJoinSubstitution(
            [pkg_share, 'config', 'optimizer_params.yaml'])
        return [Node(
            package='mmp_vehicle_dynamics',
            executable='dynamics_sim_node',
            name='dynamics_sim_node',
            output='screen',
            parameters=[optimizer_params],
        )]
    if choice == 'vehicle_sim':
        # Reserved slot: the external follower package was removed from the
        # workspace pending its restructure (see docstring) — announce the
        # reason instead of spawning a node that cannot exist.
        return [LogInfo(msg='[rviz_path_manager] follower:=vehicle_sim is '
                            'reserved — the external vehicle-sim follower is '
                            'being restructured; no follower started.')]
    return [LogInfo(msg=f'[rviz_path_manager] unknown follower "{choice}" '
                        f'(expected shared_3dof|none|vehicle_sim) — skipping.')]


def generate_launch_description():
    """
    RViz simulation launch file for path_manager

    Usage:
        ros2 launch path_manager rviz_path_manager.launch.py
    """

    pkg_share = FindPackageShare('path_manager')
    path_manager_launch = PathJoinSubstitution([pkg_share, 'launch', 'path_manager.launch.py'])

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
        DeclareLaunchArgument(
            'debug',
            default_value=EnvironmentVariable('MMP_DEBUG', default_value='0'),
            description='Debug mode passthrough (see path_manager.launch.py)'
        ),
        DeclareLaunchArgument(
            'follower',
            default_value='shared_3dof',
            description='Dynamics follower: shared_3dof (C++, same model as '
                        'the optimizer) | none | vehicle_sim (reserved for '
                        'the restructured external follower).'
        ),
        # Include base path_manager launch with RViz defaults
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path_manager_launch),
            launch_arguments={
                'debug': LaunchConfiguration('debug'),
                'drone_id': LaunchConfiguration('drone_id'),
                'record_bag': LaunchConfiguration('record_bag'),
                'disable_file_logging': LaunchConfiguration('disable_file_logging'),
                'world': LaunchConfiguration('world'),
            }.items()
        ),

        OpaqueFunction(function=_follower),
    ])
