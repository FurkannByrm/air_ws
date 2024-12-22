import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    declare_operation_mode = DeclareLaunchArgument(
        name="operation_mode",
        default_value="AUTONOMOUS",
        description="Robot operation mode: AUTONOMOUS, MAPPING, MANUAL, etc."
    )

    declare_use_slam = DeclareLaunchArgument(
        name="use_slam",
        default_value="false",
        description="Enable SLAM for mapping."
    )

    # Get launch configurations
    use_slam = LaunchConfiguration("use_slam")

    # Robot description from xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("air_description"), "urdf", "air.urdf.xacro"])
    ])
    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )


    # Laser scanner launch
    merger_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sicks300_ros2_scan_merger"),
                "launch", "sicks300_scanner.launch.py"
            )
        )
    )

    # Navigation launch (conditional)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hamal_navigation"), "launch", "hamal_navigation.launch.py"
            )
        )
    )

    # SLAM launch (conditional)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hamal_mapping"), "launch", "hamal_mapping.launch.py"
            )
        ),
        condition=IfCondition(use_slam)
    )

    # Create launch description and add actions
    ld = LaunchDescription()

    # Add declared arguments
    ld.add_action(declare_operation_mode)
    ld.add_action(declare_use_slam)

    # Add nodes and launch files
    ld.add_action(robot_state_publisher)
    ld.add_action(merger_launch)
    # ld.add_action(navigation_launch)
    ld.add_action(slam_launch)

    return ld