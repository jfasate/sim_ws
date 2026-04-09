import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lpv_mpc'),
        'config',
        'lpv_mpc_params.yaml')

    lpv_mpc_node = Node(
        package='lpv_mpc',
        executable='lpv_mpc_node',
        name='lpv_mpc_node',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([lpv_mpc_node])
