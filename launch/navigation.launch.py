import os
from launch import LaunchDescription
import yaml
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

params = os.path.join(
    get_package_share_directory('farmbot_navigation'),
    'config',
    'params.yaml'
)

global_params = yaml.safe_load(open(params))['global']['ros__parameters']

def generate_launch_description():
    ld = LaunchDescription()

    path_server = Node(
        package='farmbot_navigation',
        executable='path_server',
        name='path_server',
        parameters=[params, global_params]
    )

    ld.add_action(path_server)

    return ld