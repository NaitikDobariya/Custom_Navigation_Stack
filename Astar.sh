#!/bin/bash
set -e

# === Default values ===
BUILD=0

# === Parse arguments ===
for arg in "$@"
do
    case $arg in
        --build=1)
        BUILD=1
        shift
        ;;
        --build=0)
        BUILD=0
        shift
        ;;
        *)
        echo "Unknown option: $arg"
        ;;
    esac
done

# === Conditionally build workspace ===
if [ "$BUILD" -eq 1 ]; then
    echo ">>> Building workspace..."
    colcon build
    source install/setup.bash
else
    echo ">>> Skipping build..."
    source install/setup.bash
fi

# === Set TurtleBot model ===
export TURTLEBOT3_MODEL=burger

# === Open 4 gnome-terminal windows ===
gnome-terminal -- bash -c "source ~/turtlebot3_ws/install/setup.bash; export TURTLEBOT3_MODEL=burger; ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py x_pose:=0 y_pose:=0; exec bash"
# gnome-terminal -- bash -c "source ~/turtlebot3_ws/install/setup.bash; export TURTLEBOT3_MODEL=burger; ros2 launch turtlebot_move localization_only.launch.py; exec bash"
gnome-terminal -- bash -c "source ~/turtlebot3_ws/install/setup.bash; export TURTLEBOT3_MODEL=burger; ros2 launch turtlebot_move custom_localization.launch.py; exec bash"
gnome-terminal -- bash -c "source ~/turtlebot3_ws/install/setup.bash; export TURTLEBOT3_MODEL=burger; ros2 run turtlebot_move a_star_planner; exec bash"
gnome-terminal -- bash -c "source ~/turtlebot3_ws/install/setup.bash; export TURTLEBOT3_MODEL=burger; ros2 run turtlebot_move path_follower; exec bash"

#x_pose:=-29.027692794799805 y_pose:=-15.224211692810059
