from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, TimerAction, \
    OpaqueFunction, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnExecutionComplete
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node, SetUseSimTime, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml
import os



def launch_setup(context, *args, **kwargs):
    # perform(context) returns arg as a string, hence the conversion
    # var_rl is a ROS launch type object
    # var_py is a converted version, python friendly
    start_rviz_rl = LaunchConfiguration('start_rviz')
    start_rviz_py = start_rviz_rl.perform(context) == 'true'
    fake_rl = LaunchConfiguration('fake')
    fake_py = fake_rl.perform(context) == 'true'
    gazebo_rl = LaunchConfiguration('gazebo')
    gazebo_py = gazebo_rl.perform(context) == 'true'


    # config
    config_file_rl = LaunchConfiguration('config_file')
    config_file_py = config_file_rl.perform(context)


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('orbita3d_description'), 'urdf', 'test_orbita3d.urdf.xacro']
            ),
            *((' ', 'use_fake_hardware:=true', ' ') if fake_py else
              (' ', 'use_fake_hardware:=true use_gazebo:=true', ' ') if gazebo_py else
              (' ',)),

            ' ',
            'config_file:="{}"'.format(
                config_file_py),
            ' ',

        ]
    )  # To be cleaned on issue #92
    # print(robot_description_content.perform(context=context))

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('orbita3d_description'),
            'config',
            'test_controllers.yaml',
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('orbita3d_description'), 'config', 'test.rviz']
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='screen',
    )


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(start_rviz_rl),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager',
                   '/controller_manager'],
    )

    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller', '-c', '/controller_manager'],
    )

    forward_torque_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_torque_controller', '-c', '/controller_manager'],
    )

    forward_torque_limit_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_torque_limit_controller', '-c', '/controller_manager'],
    )

    forward_speed_limit_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_speed_limit_controller', '-c', '/controller_manager'],
    )

    forward_pid_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_pid_controller', '-c', '/controller_manager'],
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
    )


    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                forward_position_controller_spawner,
                forward_torque_controller_spawner,
                forward_torque_limit_controller_spawner,
                forward_speed_limit_controller_spawner,
                forward_pid_controller_spawner,
            ],
        ),
    )


    return [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]


def generate_launch_description():
    return LaunchDescription([
        # Needed by camera publisher - See: https://github.com/ros2/rosidl_python/issues/79
        SetEnvironmentVariable('PYTHONOPTIMIZE', '1'),

        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
            choices=['true', 'false']
        ),
        DeclareLaunchArgument(
            'fake',
            default_value='true',
            description='Start on fake_reachy mode with this launch file.',
            choices=['true', 'false']
        ),
        DeclareLaunchArgument(
            'gazebo',
            default_value='false',
            description='Start a fake_hardware with gazebo as simulation tool.',
            choices=['true', 'false']
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='fake.yaml',
            description='Orbita3d yaml config file',
        ),
        OpaqueFunction(function=launch_setup)
    ])

# TODO use a OnProcessIO to check whether every node has sent its 'OK' message and log accordingly ?
