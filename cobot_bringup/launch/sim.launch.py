import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    description_share = get_package_share_directory('cobot_description')
    urdf_xacro = os.path.join(description_share, 'urdf', 'cobot.urdf.xacro')
    robot_description_content = xacro.process_file(urdf_xacro).toxml()

    conda_prefix = os.environ.get('CONDA_PREFIX', '')

    # gz sim looks for system plugins (e.g. gz_ros2_control) only on
    # GZ_SIM_SYSTEM_PLUGIN_PATH; RoboStack installs them to $CONDA_PREFIX/lib
    # but doesn't add that to the search path itself.
    gz_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.path.join(conda_prefix, 'lib'),
    )

    # Forces gz-transport discovery onto loopback. Without this, machines
    # with multiple network interfaces (VPNs, docker bridges, etc.) can
    # cause gz-transport peers (e.g. `ros_gz_sim create`) to never find the
    # running gz sim server, hanging forever on "Requesting list of world
    # names."
    gz_ip = SetEnvironmentVariable('GZ_IP', '127.0.0.1')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
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
        gz_plugin_path,
        gz_ip,
        gz_sim,
        robot_state_publisher,
        spawn,
        joint_state_broadcaster_spawner,
        position_controller_spawner,
    ])
