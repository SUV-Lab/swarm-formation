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
    - missile_sim: the mmp_dynamics_sim submodule's Python 6-DoF node.
      RESERVED until its mmp_traj_msgs/mmp_mission_msgs migration patch is
      merged there — launching it before that dies on import.

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
    if choice == 'missile_sim':
        # Reserved slot. mmp_dynamics_sim still imports the deleted
        # path_manager.msg / formation_msgs packages, so launching it fails at
        # import — announce the reason instead of spawning a node that dies.
        # Enable once its message-migration patch lands
        # (docs/notes/scratch_dynsim_msg_migration.patch).
        return [LogInfo(msg='[rviz_path_manager] follower:=missile_sim is '
                            'reserved — mmp_dynamics_sim awaits its message '
                            'migration; no follower started.')]
    return [LogInfo(msg=f'[rviz_path_manager] unknown follower "{choice}" '
                        f'(expected none|missile_sim) — skipping.')]


def generate_launch_description():
    """
    RViz simulation launch file for path_manager
    Automatically sets: enable_visualization=true

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
                        'the optimizer) | none | missile_sim (reserved until '
                        'the mmp_dynamics_sim message-migration patch lands).'
        ),
        # Include base path_manager launch with RViz defaults
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path_manager_launch),
            launch_arguments={
                'enable_visualization': 'true',
                'debug': LaunchConfiguration('debug'),
                'drone_id': LaunchConfiguration('drone_id'),
                'record_bag': LaunchConfiguration('record_bag'),
                'disable_file_logging': LaunchConfiguration('disable_file_logging'),
                'world': LaunchConfiguration('world'),
            }.items()
        ),

        OpaqueFunction(function=_follower),
    ])
