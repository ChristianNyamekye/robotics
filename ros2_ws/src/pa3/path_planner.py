#!/usr/bin/env python

# Author: Christian Nyamekye
# Date: 2026-05-03
# Course: COSC 81/281 - Principles of Robot Design and Programming
# Assignment: PA3 - Path Planning and Global Navigation

"""
Path planner for the Stage maze world.

Subscribes to /map, publishes a Gaussian-smoothed copy on /map_smoothed,
plans a path with BFS, DFS, or A* from the robot's TF pose to a
user-supplied goal cell, publishes the result as PoseArray on
/pose_sequence, and drives the robot through the waypoints on /cmd_vel.
"""

import math
import time
import heapq
from collections import deque
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import Twist, Pose, PoseArray, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


CMD_VEL_TOPIC = 'cmd_vel'
MAP_TOPIC = 'map'
SMOOTHED_MAP_TOPIC = 'map_smoothed'
POSE_SEQUENCE_TOPIC = 'pose_sequence'
MAP_FRAME = 'map'
BASE_FRAME = 'base_link'

CONTROL_RATE = 20.0
DEFAULT_LINEAR_VELOCITY = 0.2
DEFAULT_ANGULAR_VELOCITY = math.pi / 4
DEFAULT_KP_LINEAR = 0.6
DEFAULT_KP_ANGULAR = 1.5
DEFAULT_MAX_LINEAR = 0.30
DEFAULT_MAX_ANGULAR = 1.5
DEFAULT_POSITION_TOLERANCE = 0.10
DEFAULT_HEADING_TOLERANCE = 0.08
DEFAULT_OCCUPANCY_THRESHOLD = 50
DEFAULT_BLUR_KERNEL = 5
DEFAULT_EIGHT_CONNECTED = False
DEFAULT_WEIGHTED_ASTAR = False
DEFAULT_WEIGHT_GAIN = 5.0
DEFAULT_WAYPOINT_STRIDE = 6  # subsample path: 6 * 0.05m = 0.3m between waypoints
# Robot half-width is ~0.25 m (rosbot.inc) at 0.05 m/cell -> 5 cells. We use
# 7 cells (35 cm) to give the P-controller margin for heading overshoot
# without grinding into walls. The 5x5 Gaussian alone gives at most ~1 cell
# of effective inflation, so we additionally dilate for the planning grid.
DEFAULT_INFLATION_RADIUS_CELLS = 7

USE_SIM_TIME = True
STARTUP_TIMEOUT = 15.0
TF_TIMEOUT = 0.1  # s, in-loop lookup; buffer warmed via _wait_for_tf


def gaussian_kernel(size):
    """Return a normalized Gaussian-style kernel of the given odd size (3 or 5)."""
    if size == 3:
        k = [[1, 2, 1], [2, 4, 2], [1, 2, 1]]
        s = 16.0
    elif size == 5:
        k = [
            [1,  4,  6,  4, 1],
            [4, 16, 24, 16, 4],
            [6, 24, 36, 24, 6],
            [4, 16, 24, 16, 4],
            [1,  4,  6,  4, 1],
        ]
        s = 256.0
    else:
        raise ValueError('blur_kernel_size must be 3 or 5')
    return [[v / s for v in row] for row in k]


def dilate_obstacles(data, w, h, radius, threshold):
    """
    Mark every cell within Chebyshev `radius` of any cell at or above `threshold`
    (or of unknown cells, value < 0) as fully occupied (100). Used to expand
    walls so paths keep the robot body clear.
    """
    if radius <= 0:
        return list(data)
    out = list(data)
    for r in range(h):
        row_off = r * w
        for c in range(w):
            v = data[row_off + c]
            if v < 0 or v >= threshold:
                lo_r = max(0, r - radius)
                hi_r = min(h - 1, r + radius)
                lo_c = max(0, c - radius)
                hi_c = min(w - 1, c + radius)
                for nr in range(lo_r, hi_r + 1):
                    nrow = nr * w
                    for nc in range(lo_c, hi_c + 1):
                        if out[nrow + nc] < 100:
                            out[nrow + nc] = 100
    return out


def convolve_grid(data, kernel, w, h):
    """2D convolution with edge-clamped padding. Unknown cells (-1) treated as fully occupied."""
    out = [0.0] * (w * h)
    kh = len(kernel)
    kw = len(kernel[0])
    pad_h = kh // 2
    pad_w = kw // 2
    for y in range(h):
        for x in range(w):
            acc = 0.0
            for ky in range(kh):
                sy = max(0, min(h - 1, y + ky - pad_h))
                row_off = sy * w
                k_row = kernel[ky]
                for kx in range(kw):
                    sx = max(0, min(w - 1, x + kx - pad_w))
                    val = data[row_off + sx]
                    if val < 0:
                        val = 100
                    acc += val * k_row[kx]
            out[y * w + x] = acc
    return out


def world_to_grid(x, y, info):
    col = int((x - info.origin.position.x) / info.resolution)
    row = int((y - info.origin.position.y) / info.resolution)
    return col, row


def grid_to_world(col, row, info):
    x = info.origin.position.x + (col + 0.5) * info.resolution
    y = info.origin.position.y + (row + 0.5) * info.resolution
    return x, y


def cell_free(data, w, h, col, row, threshold):
    if not (0 <= col < w and 0 <= row < h):
        return False
    val = data[row * w + col]
    if val < 0:
        return False
    return val <= threshold


def neighbors(col, row, eight):
    nb = [(col + 1, row), (col - 1, row), (col, row + 1), (col, row - 1)]
    if eight:
        nb += [
            (col + 1, row + 1), (col - 1, row + 1),
            (col + 1, row - 1), (col - 1, row - 1),
        ]
    return nb


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def reconstruct(came_from, end):
    path = []
    cur = end
    while cur is not None:
        path.append(cur)
        cur = came_from.get(cur)
    return list(reversed(path))


def bfs(data, w, h, start, goal, threshold, eight=False):
    if not cell_free(data, w, h, *goal, threshold):
        return None, 0
    queue = deque([start])
    came_from = {start: None}
    expansions = 0
    while queue:
        cur = queue.popleft()
        expansions += 1
        if cur == goal:
            return reconstruct(came_from, cur), expansions
        for n in neighbors(cur[0], cur[1], eight):
            if n in came_from or not cell_free(data, w, h, n[0], n[1], threshold):
                continue
            came_from[n] = cur
            queue.append(n)
    return None, expansions


def dfs(data, w, h, start, goal, threshold, eight=False):
    if not cell_free(data, w, h, *goal, threshold):
        return None, 0
    stack = [start]
    came_from = {start: None}
    expansions = 0
    while stack:
        cur = stack.pop()
        expansions += 1
        if cur == goal:
            return reconstruct(came_from, cur), expansions
        for n in neighbors(cur[0], cur[1], eight):
            if n in came_from or not cell_free(data, w, h, n[0], n[1], threshold):
                continue
            came_from[n] = cur
            stack.append(n)
    return None, expansions


def astar(data, w, h, start, goal, threshold, eight=False, weights=None, weight_gain=0.0):
    if not cell_free(data, w, h, *goal, threshold):
        return None, 0
    g_score = {start: 0.0}
    came_from = {start: None}
    counter = 0
    open_pq = [(manhattan(start, goal), counter, start)]
    closed = set()
    expansions = 0
    while open_pq:
        _, _, cur = heapq.heappop(open_pq)
        if cur in closed:
            continue
        closed.add(cur)
        expansions += 1
        if cur == goal:
            return reconstruct(came_from, cur), expansions
        for n in neighbors(cur[0], cur[1], eight):
            if not cell_free(data, w, h, n[0], n[1], threshold):
                continue
            step = math.hypot(n[0] - cur[0], n[1] - cur[1])
            if weights is not None and weight_gain > 0.0:
                step *= 1.0 + weight_gain * (weights[n[1] * w + n[0]] / 100.0)
            tentative = g_score[cur] + step
            if tentative < g_score.get(n, float('inf')):
                g_score[n] = tentative
                came_from[n] = cur
                counter += 1
                heapq.heappush(open_pq, (tentative + manhattan(n, goal), counter, n))
    return None, expansions


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, USE_SIM_TIME,
        )])

        self.declare_parameter('blur_kernel_size', DEFAULT_BLUR_KERNEL)
        self.declare_parameter('occupancy_threshold', DEFAULT_OCCUPANCY_THRESHOLD)
        self.declare_parameter('linear_velocity', DEFAULT_LINEAR_VELOCITY)
        self.declare_parameter('angular_velocity', DEFAULT_ANGULAR_VELOCITY)
        self.declare_parameter('kp_linear', DEFAULT_KP_LINEAR)
        self.declare_parameter('kp_angular', DEFAULT_KP_ANGULAR)
        self.declare_parameter('max_linear', DEFAULT_MAX_LINEAR)
        self.declare_parameter('max_angular', DEFAULT_MAX_ANGULAR)
        self.declare_parameter('position_tolerance', DEFAULT_POSITION_TOLERANCE)
        self.declare_parameter('heading_tolerance', DEFAULT_HEADING_TOLERANCE)
        self.declare_parameter('eight_connected', DEFAULT_EIGHT_CONNECTED)
        self.declare_parameter('weighted_astar', DEFAULT_WEIGHTED_ASTAR)
        self.declare_parameter('weight_gain', DEFAULT_WEIGHT_GAIN)
        # Smoothed map erodes thin walls (5x5 Gaussian center weight is only
        # 14% of a wall's value), so we plan on a max(raw, smoothed) grid by
        # default - walls preserved, inflation halos added on top.
        self.declare_parameter('use_smoothed_for_planning', False)
        self.declare_parameter('waypoint_stride', DEFAULT_WAYPOINT_STRIDE)
        self.declare_parameter('inflation_radius_cells', DEFAULT_INFLATION_RADIUS_CELLS)

        # /map is published with TRANSIENT_LOCAL durability by nav2_map_server.
        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self._map_sub = self.create_subscription(
            OccupancyGrid, MAP_TOPIC, self._map_callback, latched_qos
        )
        self._smoothed_pub = self.create_publisher(
            OccupancyGrid, SMOOTHED_MAP_TOPIC, latched_qos
        )
        self._pose_pub = self.create_publisher(PoseArray, POSE_SEQUENCE_TOPIC, 10)
        self._cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._odom_sub = self.create_subscription(
            Odometry, 'odom', self._odom_callback, 10
        )
        self._latest_odom = None
        self._map_to_odom_xy = None
        self._map_to_odom_yaw = 0.0

        self._raw_map = None
        self._smoothed = None

    def _odom_callback(self, msg):
        self._latest_odom = msg

    def _map_callback(self, msg):
        self._raw_map = msg
        self._smoothed = self._smooth_and_publish(msg)
        self.get_logger().info(
            f'Received map: {msg.info.width} x {msg.info.height} @ {msg.info.resolution} m/cell'
        )

    def _smooth_and_publish(self, raw):
        kernel_size = self.get_parameter('blur_kernel_size').value
        kernel = gaussian_kernel(kernel_size)
        info = raw.info
        raw_list = list(raw.data)
        smoothed = convolve_grid(raw_list, kernel, info.width, info.height)
        # Preserve walls at full value (raw) and overlay smoothing halos. A
        # single-cell wall convolved through a 5x5 Gaussian drops to ~14, which
        # is below typical occupancy thresholds; without this max() we would
        # erode thin walls into apparent free space.
        inflated = []
        for r, s in zip(raw_list, smoothed):
            r_val = 100 if r < 0 else r
            inflated.append(max(0, min(100, max(r_val, int(round(s))))))

        out = OccupancyGrid()
        out.header = raw.header
        out.info = info
        out.data = inflated
        self._smoothed_pub.publish(out)
        return inflated

    def _wait_for_map(self, timeout_sec):
        start = time.monotonic()
        while rclpy.ok() and self._raw_map is None:
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.monotonic() - start >= timeout_sec:
                self.get_logger().error('No /map received within timeout. Is nav2_map_server running?')
                return False
        return True

    def _wait_for_tf(self, timeout_sec):
        """Pump the executor until map->odom static TF is captured AND /odom arrives."""
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._map_to_odom_xy is None and self._tf_buffer.can_transform(
                MAP_FRAME, 'odom', Time()
            ):
                try:
                    t = self._tf_buffer.lookup_transform(
                        MAP_FRAME, 'odom', Time(),
                        timeout=Duration(seconds=0.1),
                    )
                    self._map_to_odom_xy = (
                        t.transform.translation.x,
                        t.transform.translation.y,
                    )
                    self._map_to_odom_yaw = quaternion_to_yaw(t.transform.rotation)
                except tf2_ros.TransformException:
                    pass
            if self._map_to_odom_xy is not None and self._latest_odom is not None:
                return True
        return False

    def _robot_pose_in_map(self):
        # Use live /odom instead of TF buffer to avoid staleness. Compose with
        # the cached static map->odom transform (captured once at startup).
        if self._latest_odom is None or self._map_to_odom_xy is None:
            return None
        o = self._latest_odom.pose.pose
        ox, oy = self._map_to_odom_xy
        oyaw = self._map_to_odom_yaw
        cos_y, sin_y = math.cos(oyaw), math.sin(oyaw)
        # Rotate odom-frame point by map->odom yaw, then translate.
        mx = ox + cos_y * o.position.x - sin_y * o.position.y
        my = oy + sin_y * o.position.x + cos_y * o.position.y
        yaw = oyaw + quaternion_to_yaw(o.orientation)
        return mx, my, yaw

    def run_session(self):
        if not self._wait_for_map(STARTUP_TIMEOUT):
            return
        if not self._wait_for_tf(STARTUP_TIMEOUT):
            self.get_logger().error(
                f'TF map->{BASE_FRAME} not available. Is static_transform_publisher running?'
            )
            return
        rclpy.spin_once(self, timeout_sec=0.1)

        print('\n============================================')
        print('  PA3 - Path Planner')
        print('  1) BFS')
        print('  2) DFS')
        print('  3) A*')
        choice = input('Select algorithm [1-3]: ').strip()
        rclpy.spin_once(self, timeout_sec=0.05)

        try:
            gx = float(input('Goal x (map frame, m): '))
            rclpy.spin_once(self, timeout_sec=0.05)
            gy = float(input('Goal y (map frame, m): '))
            rclpy.spin_once(self, timeout_sec=0.05)
        except ValueError:
            print('Invalid coordinate.')
            return

        pose = self._robot_pose_in_map()
        if pose is None:
            return
        rx, ry, ryaw = pose
        info = self._raw_map.info
        start_cell = world_to_grid(rx, ry, info)
        goal_cell = world_to_grid(gx, gy, info)

        threshold = self.get_parameter('occupancy_threshold').value
        eight = self.get_parameter('eight_connected').value
        use_smoothed = self.get_parameter('use_smoothed_for_planning').value
        inflation = int(self.get_parameter('inflation_radius_cells').value)
        base_grid = self._smoothed if use_smoothed else list(self._raw_map.data)
        # Dilate obstacles by robot's footprint so paths keep the body clear of
        # walls. The 5x5 Gaussian alone produces only ~1 cell of effective
        # inflation, which is much less than the rosbot's half-width of ~5 cells.
        plan_grid = dilate_obstacles(
            base_grid, info.width, info.height, inflation, threshold,
        )

        if not cell_free(plan_grid, info.width, info.height,
                         start_cell[0], start_cell[1], threshold):
            self.get_logger().error(
                f'Start cell {start_cell} is occupied. Robot may be inside an obstacle in the smoothed map; '
                f'try lowering occupancy_threshold or disabling use_smoothed_for_planning.'
            )
            return

        algos = {'1': 'bfs', '2': 'dfs', '3': 'astar'}
        algo = algos.get(choice)
        if algo is None:
            print('Invalid choice.')
            return

        t0 = time.monotonic()
        if algo == 'bfs':
            path, expansions = bfs(plan_grid, info.width, info.height,
                                   start_cell, goal_cell, threshold, eight=eight)
        elif algo == 'dfs':
            path, expansions = dfs(plan_grid, info.width, info.height,
                                   start_cell, goal_cell, threshold, eight=eight)
        else:
            weighted = self.get_parameter('weighted_astar').value
            weight_gain = self.get_parameter('weight_gain').value if weighted else 0.0
            weights = self._smoothed if weighted else None
            path, expansions = astar(plan_grid, info.width, info.height,
                                     start_cell, goal_cell, threshold,
                                     eight=eight, weights=weights,
                                     weight_gain=weight_gain)
        elapsed = time.monotonic() - t0

        if path is None:
            self.get_logger().warn(f'{algo.upper()} found no path. Expansions={expansions}.')
            return

        self.get_logger().info(
            f'{algo.upper()} ok: path_len={len(path)} cells, expansions={expansions}, '
            f'planning_time={elapsed*1000:.1f} ms'
        )

        pose_array = self._build_pose_array(path, info)
        self._pose_pub.publish(pose_array)
        self._drive_path(pose_array)

    def _build_pose_array(self, path, info):
        # Drop the start cell - the robot is already there; keeping it forces
        # a useless backward rotation toward (start - epsilon).
        if len(path) > 1:
            path = path[1:]
        # Subsample so waypoints are far enough apart (default 0.3 m) that
        # bearing-to-next-waypoint is stable under odom noise. Goal cell is
        # always retained.
        stride = max(1, int(self.get_parameter('waypoint_stride').value))
        if stride > 1 and len(path) > 1:
            sampled = path[::stride]
            if sampled[-1] != path[-1]:
                sampled.append(path[-1])
            path = sampled

        pa = PoseArray()
        pa.header.frame_id = MAP_FRAME
        pa.header.stamp = self.get_clock().now().to_msg()
        for i, (col, row) in enumerate(path):
            x, y = grid_to_world(col, row, info)
            if i + 1 < len(path):
                nx, ny = grid_to_world(path[i + 1][0], path[i + 1][1], info)
                yaw = math.atan2(ny - y, nx - x)
            elif pa.poses:
                yaw = quaternion_to_yaw(pa.poses[-1].orientation)
            else:
                yaw = 0.0
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.orientation = yaw_to_quaternion(yaw)
            pa.poses.append(p)
        return pa

    def _drive_path(self, pose_array):
        for i, p in enumerate(pose_array.poses):
            gx, gy = p.position.x, p.position.y
            self.get_logger().info(f'-> waypoint {i+1}/{len(pose_array.poses)} ({gx:.2f}, {gy:.2f})')
            if not self._drive_to_point(gx, gy):
                self.get_logger().warn('Drive aborted.')
                return
        self.stop()
        self.get_logger().info('Path complete.')

    def _drive_to_point(self, gx, gy):
        kp_a = self.get_parameter('kp_angular').value
        kp_l = self.get_parameter('kp_linear').value
        max_l = self.get_parameter('max_linear').value
        max_a = self.get_parameter('max_angular').value
        pos_tol = self.get_parameter('position_tolerance').value
        head_tol = self.get_parameter('heading_tolerance').value

        dt = 1.0 / CONTROL_RATE

        # Skip if we're already inside the position tolerance - avoids the
        # rotate-toward-current-cell-then-back-out trap.
        pose = self._robot_pose_in_map()
        if pose is None:
            return False
        if math.hypot(gx - pose[0], gy - pose[1]) < pos_tol:
            return True

        # Phase 1: rotate to face goal.
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            pose = self._robot_pose_in_map()
            if pose is None:
                return False
            rx, ry, ryaw = pose
            target_yaw = math.atan2(gy - ry, gx - rx)
            err = normalize_angle(target_yaw - ryaw)
            if abs(err) < head_tol:
                break
            cmd = max(-max_a, min(max_a, kp_a * err))
            self._publish(0.0, cmd)
            time.sleep(dt)

        # Phase 2: drive to goal with mild heading correction.
        # Stuck-detection: if dist hasn't dropped by >1cm in 3s, advance to
        # next waypoint. Better than grinding against an unmodeled obstacle.
        last_progress_time = time.monotonic()
        last_dist = float('inf')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            pose = self._robot_pose_in_map()
            if pose is None:
                return False
            rx, ry, ryaw = pose
            dx = gx - rx
            dy = gy - ry
            dist = math.hypot(dx, dy)
            if dist < pos_tol:
                break
            if last_dist - dist > 0.01:
                last_progress_time = time.monotonic()
                last_dist = dist
            elif time.monotonic() - last_progress_time > 3.0:
                self.get_logger().warn(
                    f'Stuck near ({rx:.2f},{ry:.2f}); skipping to next waypoint.'
                )
                break
            target_yaw = math.atan2(dy, dx)
            head_err = normalize_angle(target_yaw - ryaw)
            v = min(max_l, kp_l * dist)
            if abs(head_err) > math.pi / 6:
                v = 0.0
            w = max(-max_a, min(max_a, kp_a * head_err))
            self._publish(v, w)
            time.sleep(dt)

        self._publish(0.0, 0.0)
        return True

    def _publish(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self._cmd_pub.publish(t)

    def stop(self):
        self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PathPlanner()
    try:
        node.run_session()
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received. Stopping.')
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
