# Custom Autonomous Navigation Stack for ROS2

A fully functional, from-scratch implementation of an autonomous navigation system for differential-drive mobile robots operating in known indoor environments, algorithm was developed using Tuetlebot3 Burger simulation model in Gazebo. This stack integrates three core components: laser-based localization, global path planning, and trajectory tracking control. This was then implemented in an actual mobile robot developed by me called The White Robot and the results are presented. 

The Navigation stack and the White Robot are in going to be in a continuous state  of improvement. The plan is also to make the White Robot an open-source alternative to the Turtlebot3 Burger model, and the progress and materials will be shared continuously in the future. 

## Overview

This Navigation stack implements a complete navigation pipeline enabling robots to autonomously localize themselves on a known occupancy grid map, plan collision-free paths to user-specified goals, and execute smooth trajectory tracking. The system demonstrates robust performance in Gazebo simulations and RViz visualizations, providing a production-quality foundation for mobile robot navigation. The video is as follows.

[![Watch Video 1](https://img.youtube.com/vi/jNI1nhBfpCg/maxresdefault.jpg)](https://youtu.be/jNI1nhBfpCg)


The White Robot is made with off-the-shelf materials and 3D printed components, the CAD files of the components, the list of materials used, and a complete video tutorial of assembling the robot is planned and released as quickly as possible. The bot is still going under development so it does have any support for ROS2 so the navigation stack we ran on Gazebo need to be run in some way in this bot, to do that, we run the localization, planning, and navigation nodes as simple python scripts and send commands to the motors. The results are presented in the video shown below.

[![Watch Video 2](https://img.youtube.com/vi/XnThzf2u9W4/hqdefault.jpg)](https://youtu.be/XnThzf2u9W4)


## Architecture

The navigation stack comprises of three independent ROS2 nodes that communicate via topics and transforms:

```
Localization Node (custom_localizer.py) ← /scan (2D laser scan)
        ↓
    /amcl_pose (pose estimation with covariance)
        ↓
Global Planner Node (a_star_planner.py) ← /map (2D occupancy map)
        ↓
    /global_path (waypoint trajectory)
        ↓
Path Follower Node (path_follower.py)
        ↓
    /cmd_vel (velocity commands to robot)
```

## Component Details

### 1. Custom Localization Node (`custom_localizer.py`)

**Purpose:** Estimates the robot's pose on a known map using 2D laser scan data.

**Approach:** Pattern matching localization by robust cost minimization. Rather than particle filtering or traditional Monte Carlo methods, this node evaluates pose candidates by computing the optimal fit between observed laser scans and the expected map structure. The best fit laser scan with the 2D occupancy grid should give us the real robot pose, that is the logic.

**How It Works:**


1. **Map Representation and distance transform:**
   - Loads a 2D occupancy grid map.
   - Converts the image to binary occupancy: occupied cells are True, anything else is False
   - Computes a Euclidean distance transform \(D(x, y)\), storing the minimum distance from each cell to the nearest obstacle, which gives us a distance map.
   - This distance map enables fast evaluation of scan-to-map consistency

2. **Pose-to-Cost Evaluation:**
   - For a candidate pose \((x, y, θ), transforms all laser scan rays into map coordinates
   - Projects each lidar endpoint onto the distance map
   - Evaluates the cost function for the given pose

   The cost for pose p = (x, y, θ) is:
   
   <img width="395" height="124" alt="image" src="https://github.com/user-attachments/assets/74c92d1c-943e-4d87-bc59-36f15bd4d320" />

   
   where r_i is the measured scan range, d_i is the distance transform value at the projected point, and L_Huber is the Huber loss function providing robustness against outliers.

3. **Search Strategy:**
   - **Coarse Search:** Samples candidate poses on a regular grid over x, y, and orientation θ. Evaluates the cost for each candidate and selects the best (minimum cost).
   - **Refinement:** Applies pattern search optimization from the coarse estimate. Iteratively tests neighbor poses by small increments in x, y, and θ. Moves to any improving neighbor; otherwise, halves step sizes and repeats. Continues until convergence or step size becomes negligible.

4. **Stability and Filtering:**
   - Compares new pose score with previous score; rejects large jumps or high-cost poses to prevent erratic jumps
   - Applies exponential moving average (EMA) smoothing to pose estimate:
   
   <img width="616" height="113" alt="image" src="https://github.com/user-attachments/assets/190c0f8f-5b06-4acc-8885-ad2f49da410a" />

   
   where alpha is the smoothing factor (typically 0.1–0.3)

5. **Output:**
   - Publishes pose on `/amcl_pose`
   - Broadcasts TF transform from "map" frame to "odom" frame

---

### 2. Global Path Planner (`a_star_planner.py`)

**Purpose:** Computes collision-free paths from the robot's current pose to user-specified goals on the occupancy grid.

**Algorithm:** A* graph search on a discretized occupancy grid.

**How It Works:**

1. **Map Processing:**
   - Loads occupancy grid 
   - Inflates obstacles by the robot's radius to ensure safe navigation margins

2. **A-star Search:**
   - Pretty standard stuff
   - Explores grid cells in order of \(f(n) = g(n) + h(n)\) where:
     - \(g(n)\) is the actual cost to reach node \(n\) from start
     - \(h(n)\) is the heuristic estimated cost from \(n\) to goal
   - Naturally avoids inflated obstacle regions for collision safety

3. **Path Optimization:**
   - Line-of-Sight Pruning: Post-processes the A* path by checking if waypoints can be connected directly
   - This reduces waypoint count and simplifies the trajectory without sacrificing safety

4. **Dynamic Replanning:**
   - Subscribes to `/amcl_pose` and goal topics (`/move_base_simple/goal`, `/goal_pose`)
   - Automatically replans whenever a new goal arrives or on a periodic timer (default 1 second)

5. **Output:**
   - Publishes optimized path as `nav_msgs/Path` on `/global_path`

---

### 3. Path Follower (`path_follower.py`)

**Purpose:** Executes the global path by generating real-time velocity commands that drive the robot along computed waypoints.

**Control Strategy:** Rotate-then-go with heading correction for robust waypoint following.

**How It Works:**

1. **State Management:**
   - Maintains current waypoint index and tracks path progress
   - Subscribes to `/global_path` (updated waypoints) and `/amcl_pose` (current robot state)
   - Stops immediately if the path is unavailable or the pose estimate is stale

2. **Control Logic at Each Iteration:**

   For the current waypoint w = (x_g, y_g) and robot pose p = (x_r, y_r, θ_r):

   **a. Compute Desired Heading:**
      
      <img width="616" height="113" alt="image" src="https://github.com/user-attachments/assets/b92e2f59-7299-4efd-a1e6-0b6862ae7488" />


   **b. Calculate Heading Error:**
      
      <img width="616" height="113" alt="image" src="https://github.com/user-attachments/assets/13dfef7a-7f0d-47f8-9a87-e0f8a8b87a02" />


   **c. Rotation Phase (if |Δθ| > θ_threshold):**
      - Generate pure rotation commands
      - Suppress forward motion until heading aligns

   **d. Motion Phase (if |Δθ| ≤ θ_threshold):**

   **e. Waypoint Advancement:**
      - When d < d_tolerance, advances to next waypoint
      - Resets control state and repeats

3. **Goal Convergence:**
   - When the final waypoint is reached, sets velocity commands to zero and publishes a stop command
   - Maintains zero velocity until the next path arrives

5. **Output:**
   - Publishes velocity commands on `/cmd_vel`

---

## Project structure

Clone the entire repository.

`
git clone https://github.com/NaitikDobariya/Custom_Navigation_Stack.git
`

1. **Navigation Stack**

   - ROS2 (tested on Jazzy)
   - Gazebo (tested on Harmonic)
   ```
   cd Custom_Naviagtion_Stack

   # build workspace.
   colcon build --symlink-install
   
   # source it.
   source install/setup.bash
   ```

   The nodes that run all the given in the `/src` folder, which is present in the workspace. The folder is at the location.

   `src/turtlebot_move/turtlebot_move`

   It contains `a_star_planner.py`, `custom_localizer.py `, and `path_follower.py`. The name itself explains the function as well.

   To run the stack, run the shell script `Astar.sh`

   ` ./Astar.sh`

   In case any packages are not found, the `ros2_packages_list.txt` can be referred to see whcih packages are needed to run the stack. Idealy there should be a Docker container where we can run this stack for a seamless running of the stack on any system, as the web of dependencies and version is very tough to navigate.

2. **White Robot**

   For the White Robot, currently I have provided all the .stl files for all the parts that need to be 3D printed in the `parts_stl_file` folder and the list of off-the-shelf parts used as listed in the `bill_of_materials.txt`.
   
---

## Key Design Decisions

**Pattern Matching vs. Particle Filtering:** This was something I felt like trying, and it seems to be working for now, but it  is hard to say whether it will beat Paticle filter based methods in a real scenario. This method could be the problem behind the test on the actual robot that was done.

**Obstacle Inflation:** The planner inflates obstacles by the robot's radius rather than treating the robot as a point. This ensures all generated paths maintain safe clearance and avoid collisions at plan time. This is done to negate the usage of a local controller. This will fail instantly in case of dynamic obstacles.

**Path Pruning:** Line-of-sight pruning removes unnecessary waypoints, effective in sparse, structured environments, where direct line of sight paths are abundant.

**Rotate-Then-Go Control:** The path follower prioritises heading alignment before motion. This is the simplest and is robust for differential-drive robots and prevents sideways drift, though it may not be optimal for omnidirectional platforms.

**EMA Filtering:** Exponential smoothing in localization balances responsiveness to true pose changes with rejection of noise-induced jitter, creating stable pose estimates. This is a temporary fix and shows the limitation of Pattern matching based localization.

---

## Project Status & Conclusion
All the sucesses have been mentioned in their respective sections above, here we will discuss what can be improved and things to be done in the future.

- A Docker setup needs to be made in order to run the stack on any computer; currently, it would be a hassle for anyone to clone the repo and run the simulation setup as it is.

- Make the robot ROS2 compatible to run the stack properly and diagnose issues with it, and provide that code setup as well.

- Add encoder and IMU for future upgrades and performance enhancements.

- Documentation and a detailed video tutorial for making the White Robot from all the parts listed and the appropriate wiring connections, diagrams, and ROS2 setup in the Raspberry Pi.

That's it, feel free to suggest changes!
