from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
import os

from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from uaclient.api.u.pro.services.dependencies.v1 import dependencies


def generate_launch_description():

    # package name
    package_name = 'kungshu_hardware_arm'

    #node name
    node_name = 'arm_node'

    node_path_installed = PathJoinSubstitution([
        FindPackagePrefix(package_name),
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
        emulate_tty=True
    )

    # 3. 注册事件：当setcap_process执行完毕后，启动arm_node
    start_arm_node_after_setcap = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=setcap_process,  # 监听setcap_process的退出事件
            on_exit=[arm_node]  # 退出后执行的动作（启动节点）
        )
    )

    return LaunchDescription([
        setcap_process,
        start_arm_node_after_setcap
    ])