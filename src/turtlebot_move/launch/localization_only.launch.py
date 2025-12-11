from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to your map
    map_file = "/home/naitik/turtlebot3_ws/src/turtlebot_move/map/rrc.yaml"

    # Use Nav2’s default AMCL params (can be tuned later)
    params_file = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "params", "nav2_params.yaml"
    )

    return LaunchDescription([
        # Map server
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": map_file}]
        ),

        # AMCL localization
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[params_file]
        ),

        # Lifecycle manager (keeps AMCL + map_server alive)
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server", "amcl"]
            }]
        ),

        # RViz2 (with default Nav2 config)
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(
                get_package_share_directory("nav2_bringup"),
                "rviz", "/home/naitik/turtlebot3_ws/src/turtlebot_move/rviz/project_Astar.rviz"
            )]
        ),
    ])
