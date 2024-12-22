from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)

from launch_ros.actions import PushRosNamespace, SetRemap


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
]


def launch_setup(context, *args, **kwargs):

    pkg_clearpath_nav2_demos = get_package_share_directory('air_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    
    file_parameters = PathJoinSubstitution([
        pkg_clearpath_nav2_demos,
        'config',
        'air_navigation.yaml'])

    launch_nav2 = PathJoinSubstitution(
      [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])

    nav2 = GroupAction([
        #PushRosNamespace(namespace),
        #SetRemap('/' + namespace + '/global_costmap/sensors/lidar2d_0/scan',
        #         '/' + namespace + '/sensors/lidar2d_0/scan'),
        #SetRemap('/' + namespace + '/local_costmap/sensors/lidar2d_0/scan',
        #         '/' + namespace + '/sensors/lidar2d_0/scan'),
        SetRemap('cmd_vel', 'hamal_base_controller/cmd_vel_unstamped'),
        #SetRemap(src='/hamal_base_controller/cmd_vel_unstamped',dst='/cmd_vel_nav'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_nav2),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('params_file', file_parameters),
                ('use_composition', 'False'),
                #('map_yaml_file', '/home/hamal22/hamal_ros2_ws/src/hamal_mapping/ayosb_maps/ayosb_arge.yaml')
                #('namespace', namespace)
              ],
        ),
    ])

    return [nav2]

def localization_launch_setup(context, *args, **kwargs):
    # Packages
    pkg_clearpath_nav2_demos = get_package_share_directory('hamal_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    map = LaunchConfiguration('map')


    file_parameters = PathJoinSubstitution([
        pkg_clearpath_nav2_demos,
        'config',
        'hamal_navigation.yaml'])

    launch_localization = PathJoinSubstitution(
      [pkg_nav2_bringup, 'launch', 'localization_launch.py'])

    localization = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_localization),
            launch_arguments=[
                ('map', map),
                ('use_sim_time', use_sim_time),
                ('params_file', file_parameters)
              ]
        ),
    ])

    return [localization]

def generate_launch_description():
  
    mapping_pkg = get_package_share_directory('air_mapping')
    map_arg = DeclareLaunchArgument(
      'map',
      default_value=PathJoinSubstitution([mapping_pkg, 'arge', 'arge.yaml']),
      description='Full path to map yaml file to load'
    )
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(map_arg)
    ld.add_action(OpaqueFunction(function=launch_setup))
    ld.add_action(OpaqueFunction(function=localization_launch_setup))
    return ld