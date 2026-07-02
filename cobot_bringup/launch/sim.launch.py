import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_share = get_package_share_directory('cobot_description')
    urdf_xacro = os.path.join(description_share, 'urdf', 'cobot.urdf.xacro')

    robot_description = ExecuteProcess(
        cmd=['xacro', urdf_xacro],
        output='screen',
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'cobot'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_position_controller'],
    )

    return LaunchDescription([
        gz_sim,
        robot_description,
        spawn,
        joint_state_broadcaster_spawner,
        position_controller_spawner,
    ])
