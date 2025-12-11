#!/usr/bin/env python3
"""
a_star_planner.py

- Loads map.yaml + .pgm (default: /home/naitik/map.yaml)
- Builds inflated occupancy grid
- Listens to /amcl_pose and RViz goal topics (/move_base_simple/goal and /goal_pose)
- Runs A* and publishes nav_msgs/Path to /global_path
- Periodic replanning (default 1.0s)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml
import math
import heapq
import time
import os
from typing import Tuple, List, Optional, Dict
import numpy as np

# ----------------------------
# Utilities: PGM loader (supports P5 binary and P2 ascii)
# ----------------------------
def load_pgm(path: str) -> np.ndarray:
    with open(path, 'rb') as f:
        header = f.readline().decode('ascii').strip()
        if header not in ('P5', 'P2'):
            raise RuntimeError(f'Unsupported PGM header: {header}')
        # skip comments
        def read_non_comment():
            line = f.readline().decode('ascii')
            while line.startswith('#'):
                line = f.readline().decode('ascii')
            return line
        dims_line = read_non_comment()
        while dims_line.strip() == '':
            dims_line = read_non_comment()
        parts = dims_line.split()
        if len(parts) == 2:
            width, height = int(parts[0]), int(parts[1])
        else:
            # maybe separated lines
            width = int(parts[0])
            height = int(read_non_comment().strip())
        maxval = int(read_non_comment().strip())
        if header == 'P5':
            # binary
            raw = f.read(width * height)
            img = np.frombuffer(raw, dtype=np.uint8).reshape((height, width))
        else:
            # ascii
            data = []
            while len(data) < width * height:
                line = read_non_comment()
                data.extend([int(x) for x in line.split()])
            img = np.array(data, dtype=np.uint8).reshape((height, width))
        return img

# ----------------------------
# Bresenham for line-of-sight in grid coordinates
# ----------------------------
def bresenham_line(x0:int, y0:int, x1:int, y1:int):
    """Yield integer grid coordinates along the line from (x0,y0) to (x1,y1)."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    if dy <= dx:
        err = dx // 2
        while x != x1:
            yield x, y
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
        yield x, y
    else:
        err = dy // 2
        while y != y1:
            yield x, y
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
        yield x, y

# ----------------------------
# A* implementation on a boolean grid (True = occupied)
# ----------------------------
def astar_grid(start: Tuple[int,int],
               goal: Tuple[int,int],
               grid: np.ndarray) -> Optional[List[Tuple[int,int]]]:
    h = lambda a, b: math.hypot(a[0]-b[0], a[1]-b[1])

    H, W = grid.shape  # grid indexed as [row=y, col=x] where y in [0,H-1]
    # quick bounds check
    sx, sy = start
    gx, gy = goal
    if not (0 <= sx < W and 0 <= sy < H and 0 <= gx < W and 0 <= gy < H):
        return None
    if grid[sy, sx] or grid[gy, gx]:
        return None

    neighbors = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,-1),(-1,1),(1,1)]
    costs = [1.0,1.0,1.0,1.0,math.sqrt(2),math.sqrt(2),math.sqrt(2),math.sqrt(2)]

    open_heap = []
    gscore: Dict[Tuple[int,int], float] = {}
    came_from: Dict[Tuple[int,int], Tuple[int,int]] = {}

    gscore[start] = 0.0
    heapq.heappush(open_heap, (h(start,goal), 0.0, start))

    visited = set()

    while open_heap:
        f, g, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            # reconstruct
            path = [current]
            while current != start:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        cx, cy = current
        for (dx,dy),c in zip(neighbors,costs):
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < W and 0 <= ny < H):
                continue
            if grid[ny, nx]:
                continue
            neighbor = (nx, ny)
            tentative_g = g + c
            if tentative_g < gscore.get(neighbor, float('inf')):
                gscore[neighbor] = tentative_g
                came_from[neighbor] = current
                heapq.heappush(open_heap, (tentative_g + h(neighbor, goal), tentative_g, neighbor))
    return None

# ----------------------------
# Path pruning by line-of-sight (Bresenham)
# ----------------------------
def prune_path(path: List[Tuple[int,int]], grid: np.ndarray) -> List[Tuple[int,int]]:
    if not path or len(path) < 3:
        return path[:]
    pruned = [path[0]]
    i = 0
    n = len(path)
    while i < n-1:
        # try to jump as far as possible
        j = n-1
        while j > i+1:
            a = path[i]
            b = path[j]
            # check LOS between a and b
            blocked = False
            for x,y in bresenham_line(a[0], a[1], b[0], b[1]):
                # bounds safety
                if y < 0 or y >= grid.shape[0] or x < 0 or x >= grid.shape[1]:
                    blocked = True
                    break
                if grid[y, x]:
                    blocked = True
                    break
            if not blocked:
                break
            j -= 1
        # j is the farthest we can reach
        pruned.append(path[j])
        i = j
    return pruned

# ----------------------------
# Node
# ----------------------------
class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')

        # Parameters
        self.declare_parameter('map_yaml_path', '/home/naitik/turtlebot3_ws/src/turtlebot_move/map/map_nw.yaml')
        self.declare_parameter('robot_radius', 0.18)
        self.declare_parameter('replan_interval', 1.0)
        self.declare_parameter('occupied_thresh', 0.65)
        self.declare_parameter('free_thresh', 0.196)
        self.declare_parameter('negate', 0)

        self.map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self.replan_interval = self.get_parameter('replan_interval').get_parameter_value().double_value
        self.occupied_thresh = self.get_parameter('occupied_thresh').get_parameter_value().double_value
        self.free_thresh = self.get_parameter('free_thresh').get_parameter_value().double_value
        self.negate = bool(self.get_parameter('negate').get_parameter_value().integer_value)

        self.get_logger().info(f'Using map yaml: {self.map_yaml_path}')
        # Load map
        if not os.path.isfile(self.map_yaml_path):
            self.get_logger().error(f'map yaml not found: {self.map_yaml_path}')
            raise RuntimeError('map yaml not found')

        with open(self.map_yaml_path, 'r') as f:
            map_yaml = yaml.safe_load(f)

        img_file = os.path.join(os.path.dirname(self.map_yaml_path), map_yaml['image'])
        self.resolution = float(map_yaml.get('resolution', 0.05))
        origin = map_yaml.get('origin', [0.0, 0.0, 0.0])
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.origin_theta = float(origin[2]) if len(origin) > 2 else 0.0
        # map_server fields if present
        if 'occupied_thresh' in map_yaml:
            self.occupied_thresh = float(map_yaml['occupied_thresh'])
        if 'free_thresh' in map_yaml:
            self.free_thresh = float(map_yaml['free_thresh'])
        if 'negate' in map_yaml:
            self.negate = bool(map_yaml['negate'])

        self.get_logger().info(f'Loading image: {img_file}')
        img = load_pgm(img_file)  # shape (H, W), dtype uint8
        H, W = img.shape
        self.map_height = H
        self.map_width = W

        # compute occupancy probability like map_server:
        # if negate: invert image first such that 0=black(occupied),255=white(free)
        if self.negate:
            img_proc = 255 - img
        else:
            img_proc = img.copy()

        occ_prob = (255 - img_proc) / 255.0  # 1.0 for black (occupied), 0.0 for white (free)
        # create binary occupied grid (True = occupied)
        occ_thresh_val = self.occupied_thresh
        free_thresh_val = self.free_thresh
        binary_grid = np.ones_like(occ_prob, dtype=bool)  # default occupied
        binary_grid[occ_prob < free_thresh_val] = False  # free
        binary_grid[occ_prob >= occ_thresh_val] = True  # occupied
        # ambiguous cells (between free and occ) -> treat as occupied for safety
        occupied = binary_grid.copy()

        # Inflate obstacles by robot radius (in pixels)
        inflation_px = int(math.ceil(self.robot_radius / self.resolution))
        self.get_logger().info(f'Inflating obstacles by {inflation_px} px ({self.robot_radius} m)')

        inflated = occupied.copy()
        if inflation_px > 0:
            ys, xs = np.where(occupied)
            # precompute offsets in disk
            offsets = []
            r = inflation_px
            for dy in range(-r, r+1):
                for dx in range(-r, r+1):
                    if dx*dx + dy*dy <= r*r:
                        offsets.append((dx, dy))
            # mark inflated
            for (x0,y0) in zip(xs, ys):
                for dx, dy in offsets:
                    x1 = x0 + dx
                    y1 = y0 + dy
                    if 0 <= x1 < self.map_width and 0 <= y1 < self.map_height:
                        inflated[y1, x1] = True
        self.occupancy = inflated  # boolean grid, shape (H, W) where (x,index) mapping is handled below

        self.get_logger().info(f'Map loaded: width={W}, height={H}, resolution={self.resolution}, origin=({self.origin_x},{self.origin_y})')

        # Publisher for path
        qos = QoSProfile(depth=10)
        self.path_pub = self.create_publisher(Path, '/global_path', qos)

        # Subscribers
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, qos)
        # support both typical rviz topics
        self.goal_sub1 = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_cb, qos)
        self.goal_sub2 = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, qos)

        # internal state
        self.current_amcl_pose = None  # PoseWithCovarianceStamped payload
        self.current_goal_pose = None  # PoseStamped payload
        self.last_published_path = None
        self.planning = False

        # Timer for replanning
        self.timer = self.create_timer(self.replan_interval, self.timer_replan)

        self.get_logger().info('A* planner ready. Waiting for /amcl_pose and goal (click in RViz).')

    # ----------------------------
    # Callbacks
    # ----------------------------
    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.current_amcl_pose = msg

    def goal_cb(self, msg: PoseStamped):
        # store goal and plan immediately
        self.current_goal_pose = msg
        self.get_logger().info(f'Received new goal (map frame): ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        # immediate plan
        self.plan_and_publish()

    # ----------------------------
    # Timer callback
    # ----------------------------
    def timer_replan(self):
        # only plan if we have both pose and goal
        if self.current_amcl_pose is None or self.current_goal_pose is None:
            return
        # plan periodically
        self.plan_and_publish()

    # ----------------------------
    # Main planning pipeline
    # ----------------------------
    def plan_and_publish(self):
        # avoid overlapping plans
        if self.planning:
            return
        self.planning = True
        try:
            # get start (from amcl) and goal (from current_goal_pose)
            start_map = self.pose_to_map_xy(self.current_amcl_pose.pose.pose.position.x,
                                            self.current_amcl_pose.pose.pose.position.y)
            goal_map = (self.current_goal_pose.pose.position.x, self.current_goal_pose.pose.position.y)
            start_idx = self.map_to_index(start_map[0], start_map[1])
            goal_idx = self.map_to_index(goal_map[0], goal_map[1])
            if start_idx is None or goal_idx is None:
                self.get_logger().warn('Start or goal out of map bounds.')
                return
            sx, sy = start_idx
            gx, gy = goal_idx
            # run A*
            t0 = time.time()
            path_idx = astar_grid((sx, sy), (gx, gy), self.occupancy)
            t1 = time.time()
            if path_idx is None:
                self.get_logger().warn(f'A* failed to find a path (start={start_idx}, goal={goal_idx})')
                # publish empty path
                empty_path = Path()
                empty_path.header.frame_id = 'map'
                empty_path.header.stamp = self.get_clock().now().to_msg()
                self.path_pub.publish(empty_path)
                return
            self.get_logger().info(f'A* found path with {len(path_idx)} nodes in {t1-t0:.3f}s')
            # prune path
            pruned_idx = prune_path(path_idx, self.occupancy)
            self.get_logger().info(f'Pruned path: {len(path_idx)} -> {len(pruned_idx)} waypoints')
            # convert to PoseStamped list in map frame
            poses = []
            for (ix, iy) in pruned_idx:
                mx, my = self.index_to_map(ix, iy)
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.pose.position.x = mx
                ps.pose.position.y = my
                ps.pose.position.z = 0.0
                # orientation left as zero (facing forward) — follower will compute heading
                ps.pose.orientation.w = 1.0
                poses.append(ps)
            path = Path()
            path.header.frame_id = 'map'
            path.header.stamp = self.get_clock().now().to_msg()
            path.poses = poses
            # publish only if changed (or always publish)
            self.path_pub.publish(path)
            self.last_published_path = path
        except Exception as e:
            self.get_logger().error(f'Exception in planning: {e}')
        finally:
            self.planning = False

    # ----------------------------
    # Coordinate transforms: map <-> grid index
    # ----------------------------
    def map_to_index(self, mx: float, my: float) -> Optional[Tuple[int,int]]:
        # convert map meter coords to grid indices (ix,iy) where ix in [0,W-1], iy in [0,H-1]
        # ix = floor((mx - origin_x) / resolution)
        # iy_image = height - 1 - floor((my - origin_y) / resolution)
        ix = int(math.floor((mx - self.origin_x) / self.resolution))
        iy_image = int(math.floor((my - self.origin_y) / self.resolution))
        iy = (self.map_height - 1) - iy_image
        if ix < 0 or ix >= self.map_width or iy < 0 or iy >= self.map_height:
            return None
        return (ix, iy)

    def index_to_map(self, ix: int, iy: int) -> Tuple[float, float]:
        # convert grid (ix,iy) to map coordinates (center of cell)
        x = self.origin_x + (ix + 0.5) * self.resolution
        y = self.origin_y + ((self.map_height - 1 - iy) + 0.5) * self.resolution
        return (x, y)

    def pose_to_map_xy(self, x: float, y: float) -> Tuple[float, float]:
        # trivial (AMCL pose is already in map frame)
        return (x, y)

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down a_star_planner')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
