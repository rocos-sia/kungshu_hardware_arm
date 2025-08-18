from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
import os

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # package name
    package_name = 'kungshu_hardware_arm'

    #node name
    node_name = 'arm_node'

    node_path_installed = PathJoinSubstitution([
        FindPackageShare(package_name),
        "lib",
        package_name,
        node_name
    ])

    setcap_process = ExecuteProcess(
        cmd=['sudo', 'setcap', 'cap_net_raw,cap_net_admin+eip', node_path_installed],
        output='screen',
        emulate_tty=True,  # 模拟终端环境，确保sudo能正常工作
        shell=False
    )

    arm_node = Node(
        package=package_name,
        executable=node_name,
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        setcap_process,
        arm_node
    ])