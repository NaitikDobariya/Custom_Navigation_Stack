# localization_replace.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    map_file = "/home/naitik/turtlebot3_ws/src/turtlebot_move/map/map_nw.yaml"   # adjust if different

    # path to your custom_localizer node (package should export it)
    # assume package name is 'custom_localization_pkg' and script installed as 'custom_localizer.py'
    custom_localizer_pkg = "turtlebot_move"

    return LaunchDescription([
        # map server
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": map_file}]
        ),

        # custom localizer (script)
        Node(
            package=custom_localizer_pkg,
            executable="custom_localizer",
            name="custom_localizer",
            output="screen",
            parameters=[{"map_yaml": map_file,
                         "downsample": 4,
                         "coarse_step": 6,
                         "theta_bins": 12}],
            emulate_tty=True
        ),

        # lifecycle manager for map_server only (keeps map_server in managed lifecycle)
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["map_server"]
            }]
        ),

        # optional: start rviz with your config (adjust path)
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", "/home/naitik/turtlebot3_ws/src/turtlebot_move/rviz/project_Astar.rviz"]
        ),
    ])
