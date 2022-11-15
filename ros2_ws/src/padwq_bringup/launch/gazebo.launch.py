from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'padwq_description'
    pkg_share_dir = FindPackageShare(pkg_name) 

    default_robot_name = 'padwq'

    # declare launch arguments
    prefix_arg = DeclareLaunchArgument(
            name='prefix',
            default_value='""',
            description='',
    )

    robot_name_arg = DeclareLaunchArgument(
        name='robot_name',
        default_value=default_robot_name,
        description='absolute path to the robot urdf file'
    )

    # initialize launch arguments
    prefix = LaunchConfiguration('prefix')
    robot_name = LaunchConfiguration('robot_name')
    
    # define nodes
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([pkg_share_dir, 'urdf', robot_name]),
        '.urdf.xacro',
        " ",
        "prefix:=",
        prefix,
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )

    gazebo_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
    )
    
    gazebo_spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', robot_name,
                '-z', '2'
            ]
    )

    controller_node = Node(
        package='padwq_bringup',
        executable='controller',
        name='controller'
    )

    ld = LaunchDescription([
        prefix_arg,
        robot_name_arg,
        robot_state_publisher_node,
        gazebo_node,
        gazebo_spawn_node,
        controller_node
    ])

    return ld
