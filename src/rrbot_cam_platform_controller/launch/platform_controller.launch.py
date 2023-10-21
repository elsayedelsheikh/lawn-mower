from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    controller_cmd = Node(package='rrbot_cam_platform_controller',
                      executable='platform_controller',
                      output='screen',
                      parameters=[{
                        'use_sim_time': False,
                      }],
                      remappings=[
                          ('/joint_trajectory', '/joint_trajectory_position_controller/joint_trajectory'),
                          ('/joint_state', '/joint_trajectory_position_controller/state')
                      ],
                      )
    ld = LaunchDescription()
    ld.add_action(controller_cmd)

    return ld
