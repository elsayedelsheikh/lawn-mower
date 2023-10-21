from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    bumpgo_cmd = Node(package='rrbot_cam_object_follower',
                      executable='object_follower',
                      output='screen',
                      parameters=[{
                        'use_sim_time': False,
                        'image_width': 640,
                        'image_height': 480,
                      }],
                      # remappings=[('/detection', '/detectnet/detections')],
                      )
    ld = LaunchDescription()
    ld.add_action(bumpgo_cmd)

    return ld
