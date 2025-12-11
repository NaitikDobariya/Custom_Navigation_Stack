#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, TransformStamped
import numpy as np
from PIL import Image
from scipy import ndimage as ndi
import yaml, os, math, time
from tf2_ros import TransformBroadcaster

# ----------------- helpers -----------------
def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q

def quaternion_to_yaw(q: Quaternion) -> float:
    w, x, y, z = q.w, q.x, q.y, q.z
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

# ----------------- Node -----------------
class CustomLocalizer(Node):
    def __init__(self):
        super().__init__('custom_localizer')

        # ---------- parameters ----------
        self.declare_parameter('map_yaml', '/home/naitik/map.yaml')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('downsample', 4)
        self.declare_parameter('coarse_step', 6)
        self.declare_parameter('theta_bins', 12)
        self.declare_parameter('coarse_margin', 8)
        self.declare_parameter('hub_delta', 1.0)
        self.declare_parameter('max_penalty', 80.0)
        self.declare_parameter('publish_cov', 0.05)

        # Stability-related parameters
        self.declare_parameter('ema_alpha', 0.4)          # smoothing weight (0..1) higher => more responsive
        self.declare_parameter('max_jump_m', 0.5)         # max allowed instantaneous jump (meters)
        self.declare_parameter('min_improvement_ratio', 0.7)  # require new score <= ratio * prev_score to accept directly
        self.declare_parameter('reject_high_cost', 5.0)   # if cost (mean loss) > this, treat as low confidence
        self.declare_parameter('scan_buffer_size', 1)     # >1: keep median of N scans to densify (set to 3 or 5 if want)

        p_map_yaml = self.get_parameter('map_yaml').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.downsample = self.get_parameter('downsample').get_parameter_value().integer_value
        self.coarse_step = self.get_parameter('coarse_step').get_parameter_value().integer_value
        self.theta_bins = self.get_parameter('theta_bins').get_parameter_value().integer_value
        self.coarse_margin = self.get_parameter('coarse_margin').get_parameter_value().integer_value
        self.hub_delta = self.get_parameter('hub_delta').get_parameter_value().double_value
        self.max_penalty = self.get_parameter('max_penalty').get_parameter_value().double_value
        self.publish_cov = self.get_parameter('publish_cov').get_parameter_value().double_value

        self.ema_alpha = float(self.get_parameter('ema_alpha').get_parameter_value().double_value)
        self.max_jump_m = float(self.get_parameter('max_jump_m').get_parameter_value().double_value)
        self.min_improvement_ratio = float(self.get_parameter('min_improvement_ratio').get_parameter_value().double_value)
        self.reject_high_cost = float(self.get_parameter('reject_high_cost').get_parameter_value().double_value)
        self.scan_buffer_size = int(self.get_parameter('scan_buffer_size').get_parameter_value().integer_value)

        # ---------- load map ----------
        if not os.path.isfile(p_map_yaml):
            self.get_logger().error(f"map yaml not found: {p_map_yaml}")
            raise RuntimeError("map yaml not found")
        with open(p_map_yaml, 'r') as f:
            mcfg = yaml.safe_load(f)
        img_rel = mcfg.get('image') or mcfg.get('map')
        img_path = img_rel if os.path.isabs(img_rel) else os.path.join(os.path.dirname(p_map_yaml), img_rel)
        if not os.path.isfile(img_path):
            self.get_logger().error(f"map image not found: {img_path}")
            raise RuntimeError("map image not found")
        img = Image.open(img_path).convert('L')
        arr = np.array(img)
        self.occ = (arr < 128).astype(np.uint8)
        self.H, self.W = self.occ.shape
        self.resolution = float(mcfg.get('resolution', 0.05))
        origin = mcfg.get('origin', [0.0, 0.0, 0.0])
        self.origin_x, self.origin_y = float(origin[0]), float(origin[1])
        self.dist = ndi.distance_transform_edt(1 - self.occ).astype(np.float32)

        # ---------- publishers & subscribers ----------
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.initpose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_cb, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------- state ----------
        self.last_scan = None
        self.user_prior = None
        self.have_pose = False

        # previous published pose (map coordinates & yaw)
        self.prev_pose_map = None   # tuple (mx,my,yaw)
        self.prev_score = None
        self.prev_time = None

        # scan buffer (for optional median accumulation)
        self._scan_buffer = []

        self.get_logger().info(f'CustomLocalizer ready. Map size: {self.W}x{self.H}, resolution {self.resolution}')

    # ---------- coordinate conversions ----------
    def map_to_pixel(self, mx, my):
        px = (mx - self.origin_x) / self.resolution
        py = (self.H - 1) - (my - self.origin_y) / self.resolution
        return px, py

    def pixel_to_map(self, px, py):
        mx = self.origin_x + (px + 0.5) * self.resolution
        my = self.origin_y + (self.H - 1 - py + 0.5) * self.resolution
        return mx, my

    # ---------- callbacks ----------
    def initialpose_cb(self, msg):
        mx, my = msg.pose.pose.position.x, msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        px, py = self.map_to_pixel(mx, my)
        self.user_prior = (px, py, yaw)
        self.get_logger().info(f"Received /initialpose -> pixel prior: ({px:.1f},{py:.1f},{math.degrees(yaw):.1f}°)")
        if self.last_scan:
            self.scan_cb(self.last_scan)

    # ---------- utilities ----------
    def huber(self, d, delta):
        r = np.abs(d)
        return np.where(r <= delta, 0.5 * r**2, delta * (r - 0.5*delta))

    def bilinear_sample(self, xs, ys):
        H, W = self.dist.shape
        ix, iy = np.floor(xs).astype(int), np.floor(ys).astype(int)
        oob = (ix < 0) | (iy < 0) | (ix >= W-1) | (iy >= H-1)
        ix, iy = np.clip(ix,0,W-2), np.clip(iy,0,H-2)
        dx, dy = xs - ix, ys - iy
        v00, v10 = self.dist[iy, ix], self.dist[iy, ix+1]
        v01, v11 = self.dist[iy+1, ix], self.dist[iy+1, ix+1]
        v0 = v00*(1-dx) + v10*dx
        v1 = v01*(1-dx) + v11*dx
        vals = v0*(1-dy) + v1*dy
        vals[oob] = self.max_penalty
        return vals

    def ranges_angles_from_scan(self, scan):
        """
        Returns (ranges, angles) where invalid/no-return beams are set to np.nan.
        """
        ranges = np.array(scan.ranges, dtype=np.float32)
        # mark invalid returns (<= range_min or >= range_max or non-finite) as NaN so they can be excluded
        invalid_mask = (ranges <= scan.range_min) | (ranges >= scan.range_max) | ~np.isfinite(ranges)
        ranges[invalid_mask] = np.nan
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        return ranges, angles

    # ---------- pose evaluation ----------
    def eval_pose_obj(self, pose_xytheta, ranges_pix, angles):
        """
        Evaluate mean robust loss for poses in pixel-space.
        ranges_pix: 1D array of ranges in pixels with np.nan for invalid beams
        angles: 1D array of beam angles (radians)
        """
        if pose_xytheta.ndim == 1: pose_xytheta = pose_xytheta.reshape(1,3)
        M, B = pose_xytheta.shape[0], ranges_pix.shape[0]
        # compute rx, ry; respects NaNs (multiplication yields NaNs where ranges_pix is NaN)
        rx = ranges_pix * np.cos(angles)
        ry = ranges_pix * np.sin(angles)

        # valid beams mask
        valid_beams = np.isfinite(ranges_pix) & (ranges_pix > 0)
        if not np.any(valid_beams):
            return np.full((M,), self.max_penalty, dtype=np.float32)

        c, s = np.cos(pose_xytheta[:,2])[:,None], np.sin(pose_xytheta[:,2])[:,None]
        xs = pose_xytheta[:,0:1] + c*rx[None,:] - s*ry[None,:]
        ys = pose_xytheta[:,1:2] + s*rx[None,:] + c*ry[None,:]
        vals = self.bilinear_sample(xs.ravel(), ys.ravel()).reshape(M,B)
        losses = self.huber(vals, self.hub_delta)

        # ignore invalid beams when averaging
        losses[:, ~valid_beams] = 0.0
        counts = float(np.sum(valid_beams))
        return np.sum(losses, axis=1) / max(1.0, counts)

    # ---------- coarse search ----------
    def coarse_search(self, ranges_pix, angles, margin=None, coarse_step=None, theta_bins=None):
        margin = margin or self.coarse_margin
        coarse_step = coarse_step or self.coarse_step
        theta_bins = theta_bins or self.theta_bins
        xs = np.arange(margin, self.W-margin, coarse_step)
        ys = np.arange(margin, self.H-margin, coarse_step)
        thetas = np.linspace(-math.pi, math.pi, theta_bins, endpoint=False)
        cand = np.array([(x,y,t) for x in xs for y in ys for t in thetas], dtype=np.float32)
        if len(cand)==0: return (self.W/2, self.H/2, 0.0), 1e12
        vals = self.eval_pose_obj(cand, ranges_pix, angles)
        idx = int(np.argmin(vals))
        return tuple(cand[idx]), float(vals[idx])

    # ---------- pattern search refinement ----------
    def refine_pattern_search(self, init_pose_pix, ranges_pix, angles):
        best = list(init_pose_pix)
        best_val = float(self.eval_pose_obj(np.array([best], dtype=np.float32), ranges_pix, angles)[0])
        step_xy, step_th, it = 4.0, 0.4, 0
        while (step_xy>0.5 or step_th>0.01) and it<60:
            neighbors = [
                (best[0]+step_xy,best[1],best[2]), (best[0]-step_xy,best[1],best[2]),
                (best[0],best[1]+step_xy,best[2]), (best[0],best[1]-step_xy,best[2]),
                (best[0],best[1],best[2]+step_th), (best[0],best[1],best[2]-step_th)
            ]
            batch = np.array(neighbors,dtype=np.float32)
            vals = self.eval_pose_obj(batch, ranges_pix, angles)
            idx = int(np.argmin(vals))
            if vals[idx] < best_val:
                best = list(batch[idx])
                best_val = vals[idx]
            else:
                step_xy *= 0.5
                step_th *= 0.5
            it += 1
        best[2] = (best[2]+math.pi)%(2*math.pi)-math.pi
        return tuple(best), best_val

    # ---------- helper: blend yaw ----------
    def _blend_yaw(self, yaw0, yaw1, alpha):
        # shortest-angle blend
        diff = ((yaw1 - yaw0 + math.pi) % (2*math.pi)) - math.pi
        return yaw0 + alpha * diff

    # ---------- publish pose & TF ----------
    def _publish_pose(self, mx, my, yaw_rad):
        # Pose message
        p = PoseWithCovarianceStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'map'
        p.pose.pose.position.x = mx
        p.pose.pose.position.y = my
        p.pose.pose.position.z = 0.0
        p.pose.pose.orientation = yaw_to_quaternion(yaw_rad)
        cov = [0.0]*36
        cov[0] = cov[7] = self.publish_cov**2
        cov[35] = 0.1**2
        p.pose.covariance = cov
        self.pose_pub.publish(p)

        # TF: map -> odom
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = mx
        t.transform.translation.y = my
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_quaternion(yaw_rad)
        self.tf_broadcaster.sendTransform(t)

    # ---------- scan callback ----------
    def scan_cb(self, scan):
        self.last_scan = scan
        ranges_m, angles = self.ranges_angles_from_scan(scan)

        # push into buffer for median denoising if enabled
        if self.scan_buffer_size > 1:
            self._scan_buffer.append(ranges_m)
            if len(self._scan_buffer) > self.scan_buffer_size:
                self._scan_buffer.pop(0)
            stacked = np.vstack(self._scan_buffer)
            # median while ignoring NaNs: use nanmedian
            ranges_m = np.nanmedian(stacked, axis=0)
        # at this point ranges_m may contain NaNs for invalid beams

        # downsample while keeping alignment with angles
        if self.downsample > 1:
            idxs = np.arange(0, len(ranges_m), self.downsample)
            ranges_m = ranges_m[idxs]
            angles = angles[idxs]

        # convert to pixels (NaNs preserved)
        ranges_pix = ranges_m / self.resolution

        # decide initial pose for refinement
        if self.user_prior is not None:
            init_pix = self.user_prior
        elif not self.have_pose:
            init_pix, _ = self.coarse_search(ranges_pix, angles)
        else:
            # when we already have a pose, bias coarse search near previous pixel pose
            if self.prev_pose_map is not None:
                # convert previous map pose to pixels and set small local grid around it
                px, py = self.map_to_pixel(self.prev_pose_map[0], self.prev_pose_map[1])
                # clamp to valid range
                px = float(np.clip(px, self.coarse_margin, self.W - self.coarse_margin - 1))
                py = float(np.clip(py, self.coarse_margin, self.H - self.coarse_margin - 1))
                # use previous yaw as starting theta
                init_pix = (px, py, self.prev_pose_map[2])
            else:
                init_pix, _ = self.coarse_search(ranges_pix, angles)

        est_pix, est_score = self.refine_pattern_search(init_pix, ranges_pix, angles)
        est_mx, est_my = self.pixel_to_map(est_pix[0], est_pix[1])
        est_yaw = est_pix[2]

        # score -> confidence (lower score = better). If score is huge, treat as low-confidence.
        score = float(est_score)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_time is None:
            dt = None
        else:
            dt = now - self.prev_time

        accept_direct = False
        if self.prev_score is None:
            accept_direct = True
        else:
            # require some improvement to accept full jump immediately
            if score <= self.min_improvement_ratio * self.prev_score:
                accept_direct = True

        # compute translation jump
        if self.prev_pose_map is not None:
            dx = est_mx - self.prev_pose_map[0]
            dy = est_my - self.prev_pose_map[1]
            jump = math.hypot(dx, dy)
        else:
            jump = 0.0

        if jump > self.max_jump_m and not accept_direct and self.prev_pose_map is not None:
            # reject big jump: blend toward previous pose instead of accepting full new pose
            alpha = self.ema_alpha * (self.max_jump_m / jump)  # smaller alpha if jump large
            mx = (1-alpha) * self.prev_pose_map[0] + alpha * est_mx
            my = (1-alpha) * self.prev_pose_map[1] + alpha * est_my
            yaw = self._blend_yaw(self.prev_pose_map[2], est_yaw, alpha)
        else:
            # accept but still smooth via EMA depending on score/confidence
            conf = max(0.0, 1.0 - min(score / self.reject_high_cost, 1.0))
            alpha = self.ema_alpha * conf
            if self.prev_pose_map is None or accept_direct:
                mx, my, yaw = est_mx, est_my, est_yaw
            else:
                mx = (1-alpha) * self.prev_pose_map[0] + alpha * est_mx
                my = (1-alpha) * self.prev_pose_map[1] + alpha * est_my
                yaw = self._blend_yaw(self.prev_pose_map[2], est_yaw, alpha)

        # publish with covariance scaled by score/confidence (temporarily adjust publish_cov)
        cov_scale = 1.0 + (score / max(1e-6, self.reject_high_cost))
        old_publish_cov = self.publish_cov
        # cap covariance multiplier so it doesn't explode
        scaled_cov = min(max(self.publish_cov, 0.001), 2.0) * cov_scale
        self.publish_cov = scaled_cov
        self._publish_pose(mx, my, yaw)
        # restore publish_cov to configured base value
        self.publish_cov = old_publish_cov

        # update prev state
        self.prev_pose_map = (mx, my, yaw)
        self.prev_score = score
        self.prev_time = now
        self.have_pose = True
        self.get_logger().info(f"Pose: x={mx:.2f}, y={my:.2f}, yaw={math.degrees(yaw):.1f}°, score={score:.3f}, jump={jump:.3f}")

# ----------------- main -----------------
def main(args=None):
    rclpy.init(args=args)
    node = CustomLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
