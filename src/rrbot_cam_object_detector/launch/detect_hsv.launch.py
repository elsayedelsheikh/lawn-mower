import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory('rrbot_cam_object_detector'),
        'config',
        'hsv_detection.yaml'
        )

    object_detector_cmd = Node(
        package='rrbot_cam_object_detector',
        executable='object_detector',
        parameters=[{'use_sim_time': False}, params_file],
        remappings=[
          ('/input_image', '/video_source/raw'),
        ],
        output='screen'
    )

    ld = LaunchDescription()

    # Add any actions
    ld.add_action(object_detector_cmd)

    return ld
