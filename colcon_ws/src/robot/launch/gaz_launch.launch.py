
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # these are the arguments you can pass this launch file, for example paused:=true
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
    )
    package_arg = DeclareLaunchArgument('urdf_package',
                                        description='The package where the robot description is located',
                                        default_value='robot')
    model_arg = DeclareLaunchArgument('urdf_package_path',
                                      description='The path to the robot description relative to the package root',
                                      default_value='urdf/robot.xacro')

    empty_world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'pause': 'true',
            'world': PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration('urdf_package')),
                'worlds',
                'house.world'  
            ])
        }.items(),
    )

      
    controllers_arg = DeclareLaunchArgument(
        'controllers_file',
        default_value='config/robot_controller.yaml',
        description='Path to controllers configuration'
    )

    description_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_package_path': LaunchConfiguration('urdf_package_path')}.items()
    )



    # push robot_description to factory and spawn robot in gazebo
    urdf_spawner_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=['-topic', '/robot_description', '-entity', 'robot', '-z', '0.5', '-unpause'],
        output='screen',
    )


    #add controller manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robot'),
                'config',
                'robot_controller.yaml'
            ])
        ],
        output='screen'
    )


    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    forward_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([  
        gui_arg,
        package_arg,
        model_arg,
        empty_world_launch,
        description_launch_py,
        controller_manager,
        urdf_spawner_node,
        joint_state_broadcaster,
        forward_position_controller
    ])