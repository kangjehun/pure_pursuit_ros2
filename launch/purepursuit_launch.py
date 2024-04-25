from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='purepursuit',
            namespace='control',
            executable='purepursuit',
            name='purepursuit',
            output='screen',
            parameters=[
                {'odometry_topic': '/odometry/base'},
                {'waypoint_topic': '/navigation/global_waypoints'},
                {'frequency': 100.0},
                {'verbose': True},
                {'lookahead_distance': 0.0},
                {'wheelbase_length': 0.475},
                {'steering_min': -25.0},
                {'steering_max': 25.0},
                {'steering_min_clipping': -25.0},
                {'steering_max_clipping': 25.0}
            ]
        )
    ])