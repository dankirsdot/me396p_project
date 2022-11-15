from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'padwq_bringup'
    pkg_share_dir = FindPackageShare(pkg_name) 
    
    pkg_desc_name = 'padwq_description'
    pkg_desc_share_dir = FindPackageShare(pkg_desc_name) 

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
        PathJoinSubstitution([pkg_desc_share_dir, 'urdf', robot_name]),
        '.urdf.xacro',
        " ",
        "prefix:=",
        prefix,
    ])
    
    robot_controllers = PathJoinSubstitution([pkg_share_dir, 'config', 'padwq_controllers.yaml'])
    
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[
            {'robot_description': robot_description},
            robot_controllers
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_description}
        ]
    )
    
    rviz_config_file = [
            PathJoinSubstitution([pkg_desc_share_dir, 'config', robot_name]),
            '.rviz'
    ]
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='both',
        arguments=['-d', rviz_config_file]
    )
    
    joint_state_broadcaster_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
    )
    
#    robot_controller_spawner = Node(
#        package="controller_manager",
#        executable="spawner",
#        arguments=["diffbot_base_controller", "-c", "/controller_manager"],
#    )

    # delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_node,
            on_exit=[rviz_node],
        )
    )

    # delay start of robot_controller after `joint_state_broadcaster`
#    delay_robot_controller_node_after_joint_state_broadcaster_node = RegisterEventHandler(
#        event_handler=OnProcessExit(
#            target_action=joint_state_broadcaster_node,
#            on_exit=[robot_controller_node],
#        )
#    )

    ld = LaunchDescription([
        prefix_arg,
        robot_name_arg,
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_node,
        delay_rviz_after_joint_state_broadcaster_node,
#        delay_robot_controller_node_after_joint_state_broadcaster_node
    ])

    return ld
