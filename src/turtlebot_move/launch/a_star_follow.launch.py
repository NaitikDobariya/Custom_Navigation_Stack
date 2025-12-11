from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/naitik/turtlebot3_ws/src/turtlebot_move/map/map.yaml', 
                'use_sim_time': True
            }]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        ),


        # AMCL localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                {'use_sim_time': True}  # set to False if running on real robot
            ]
        ),

        # Lifecycle manager to start map_server & amcl
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{   
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        ),

        # Your A* planner
        Node(
            package='turtlebot_move',
            executable='a_star_planner',
            name='a_star_planner',
            output='screen'
        ),

        # Your path follower
        Node(
            package='turtlebot_move',
            executable='path_follower',
            name='path_follower',
            output='screen'
        ),

        # RViz with copied config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/naitik/turtlebot_ws/src/turtlebot_move/rviz/tb3_navigation2.rviz']
        ),
    ])
