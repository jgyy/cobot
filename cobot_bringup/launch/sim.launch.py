import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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

    # Real UR5e description + Gazebo/ros2_control integration from upstream
    # (ur_description, ur_simulation_gz), instead of a hand-rolled arm.
    # forward_position_controller matches our existing direct-topic control
    # design (CobotTrackingEnv publishes Float64MultiArray position commands
    # directly, no MoveIt/trajectory planning in the per-step control loop).
    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('ur_simulation_gz'), '/launch/ur_sim_control.launch.py']
        ),
        launch_arguments={
            'ur_type': 'ur5e',
            'initial_joint_controller': 'forward_position_controller',
            'launch_rviz': 'false',
        }.items(),
    )

    return LaunchDescription([
        gz_plugin_path,
        gz_ip,
        ur_sim,
    ])
