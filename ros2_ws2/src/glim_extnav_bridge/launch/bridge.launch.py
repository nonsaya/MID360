from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    glim_namespace = LaunchConfiguration('glim_namespace')
    use_corrected = LaunchConfiguration('use_corrected')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    odom_child_frame_id = LaunchConfiguration('odom_child_frame_id')

    return LaunchDescription([
        DeclareLaunchArgument('glim_namespace', default_value='/glim_ros'),
        DeclareLaunchArgument('use_corrected', default_value='false'),
        DeclareLaunchArgument('publish_rate_hz', default_value='15.0'),
        DeclareLaunchArgument('odom_child_frame_id', default_value=''),

        Node(
            package='glim_extnav_bridge',
            executable='bridge_node',
            name='glim_extnav_bridge',
            output='screen',
            parameters=[{
                'glim_namespace': glim_namespace,
                'use_corrected': use_corrected,
                'publish_rate_hz': publish_rate_hz,
                'odom_child_frame_id': odom_child_frame_id,
            }]
        )
    ])


