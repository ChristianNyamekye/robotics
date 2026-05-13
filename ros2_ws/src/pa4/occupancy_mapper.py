#!/usr/bin/env python

# Author: Christian Nyamekye
# Date: 2026-05-11
# Course: COSC 81/281 - Principles of Robot Design and Programming
# Assignment: PA4 - Occupancy Grid Mapping

"""
Build a 2D occupancy grid in the odom frame from /base_scan and /odom.
Bresenham raycasting marks cells along each laser beam as free and the
endpoint cell as occupied (when the beam is a real hit). Log-odds Bayesian
update by default (binary update if use_log_odds=False). The grid grows on
demand when laser hits land outside the current bounds.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan


CMD_SCAN_TOPIC = 'base_scan'
CMD_ODOM_TOPIC = 'odom'
CMD_MAP_TOPIC = 'map'
MAP_FRAME = 'odom'

DEFAULT_RESOLUTION = 0.05
DEFAULT_WIDTH = 400
DEFAULT_HEIGHT = 400
DEFAULT_ORIGIN_X = -10.0
DEFAULT_ORIGIN_Y = -10.0

DEFAULT_PUBLISH_PERIOD = 1.0
DEFAULT_SCAN_STRIDE = 1
DEFAULT_GROWTH_PADDING = 200  # cells of headroom when expanding

DEFAULT_USE_LOG_ODDS = True
DEFAULT_L_OCC = math.log(0.65 / 0.35)
DEFAULT_L_FREE = math.log(0.35 / 0.65)
DEFAULT_L_MAX = 5.0
DEFAULT_L_MIN = -5.0
DEFAULT_L_THRESH_OCC = 0.85
DEFAULT_L_THRESH_FREE = -0.85

DEFAULT_TREAT_MAX_RANGE_AS_FREE = True

# nav_msgs/OccupancyGrid cell values (ROS spec).
OCC_UNKNOWN = -1
OCC_FREE = 0
OCC_OCCUPIED = 100

USE_SIM_TIME = True


def bresenham(x0, y0, x1, y1):
    """Integer cells along the line from (x0,y0) to (x1,y1), endpoints included."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            return
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class OccupancyMapper(Node):
    def __init__(self):
        super().__init__('occupancy_mapper')

        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, USE_SIM_TIME,
        )])

        self.declare_parameter('scan_topic', CMD_SCAN_TOPIC)
        self.declare_parameter('odom_topic', CMD_ODOM_TOPIC)
        self.declare_parameter('map_topic', CMD_MAP_TOPIC)
        self.declare_parameter('frame_id', MAP_FRAME)
        self.declare_parameter('resolution', DEFAULT_RESOLUTION)
        self.declare_parameter('width', DEFAULT_WIDTH)
        self.declare_parameter('height', DEFAULT_HEIGHT)
        self.declare_parameter('origin_x', DEFAULT_ORIGIN_X)
        self.declare_parameter('origin_y', DEFAULT_ORIGIN_Y)
        self.declare_parameter('publish_period', DEFAULT_PUBLISH_PERIOD)
        self.declare_parameter('scan_stride', DEFAULT_SCAN_STRIDE)
        self.declare_parameter('growth_padding', DEFAULT_GROWTH_PADDING)
        self.declare_parameter('use_log_odds', DEFAULT_USE_LOG_ODDS)
        self.declare_parameter('l_occ', DEFAULT_L_OCC)
        self.declare_parameter('l_free', DEFAULT_L_FREE)
        self.declare_parameter('l_max', DEFAULT_L_MAX)
        self.declare_parameter('l_min', DEFAULT_L_MIN)
        self.declare_parameter('l_thresh_occ', DEFAULT_L_THRESH_OCC)
        self.declare_parameter('l_thresh_free', DEFAULT_L_THRESH_FREE)
        self.declare_parameter('treat_max_range_as_free', DEFAULT_TREAT_MAX_RANGE_AS_FREE)

        self._resolution = self.get_parameter('resolution').value
        self._width = int(self.get_parameter('width').value)
        self._height = int(self.get_parameter('height').value)
        self._origin_x = float(self.get_parameter('origin_x').value)
        self._origin_y = float(self.get_parameter('origin_y').value)
        self._frame_id = self.get_parameter('frame_id').value
        self._use_log_odds = bool(self.get_parameter('use_log_odds').value)
        self._l_occ = float(self.get_parameter('l_occ').value)
        self._l_free = float(self.get_parameter('l_free').value)
        self._l_max = float(self.get_parameter('l_max').value)
        self._l_min = float(self.get_parameter('l_min').value)
        self._l_th_occ = float(self.get_parameter('l_thresh_occ').value)
        self._l_th_free = float(self.get_parameter('l_thresh_free').value)
        self._growth_padding = int(self.get_parameter('growth_padding').value)

        self._allocate_grid()

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._map_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('map_topic').value, map_qos,
        )
        self._scan_sub = self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._scan_callback, 10,
        )
        self._odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_callback, 10,
        )

        self._latest_odom = None
        self._scans_processed = 0
        self._growth_count = 0
        self._frame_id_adopted = False

        period = float(self.get_parameter('publish_period').value)
        self._timer = self.create_timer(period, self._publish_map)

        self.get_logger().info(
            f'Mapper running: {self._width}x{self._height} cells @ '
            f'{self._resolution} m/cell, origin '
            f'({self._origin_x:.1f},{self._origin_y:.1f}), log_odds={self._use_log_odds}'
        )

    def _allocate_grid(self):
        n = self._width * self._height
        if self._use_log_odds:
            self._log_odds = [0.0] * n
        else:
            self._binary = [OCC_UNKNOWN] * n
        self._observed = [False] * n

    def _odom_callback(self, msg):
        self._latest_odom = msg
        if not self._frame_id_adopted and msg.header.frame_id:
            if msg.header.frame_id != self._frame_id:
                self.get_logger().info(
                    f"Adopting odom frame_id '{msg.header.frame_id}' "
                    f"(default was '{self._frame_id}'). Set RViz Fixed Frame to match."
                )
                self._frame_id = msg.header.frame_id
            self._frame_id_adopted = True

    def _world_to_grid(self, x, y):
        col = int(math.floor((x - self._origin_x) / self._resolution))
        row = int(math.floor((y - self._origin_y) / self._resolution))
        return col, row

    def _in_bounds(self, col, row):
        return 0 <= col < self._width and 0 <= row < self._height

    def _idx(self, col, row):
        return row * self._width + col

    def _grow_if_needed(self, min_c, min_r, max_c, max_r):
        """Expand grid so [min_c..max_c]x[min_r..max_r] is in-bounds; return shift in cols/rows."""
        pad = self._growth_padding
        shift_c = 0
        shift_r = 0
        new_w = self._width
        new_h = self._height
        if min_c < 0:
            shift_c = -min_c + pad
            new_w += shift_c
        if min_r < 0:
            shift_r = -min_r + pad
            new_h += shift_r
        if max_c + shift_c >= new_w:
            new_w = max_c + shift_c + 1 + pad
        if max_r + shift_r >= new_h:
            new_h = max_r + shift_r + 1 + pad
        if new_w == self._width and new_h == self._height and shift_c == 0 and shift_r == 0:
            return 0, 0

        # Allocate new grid and copy old data.
        new_n = new_w * new_h
        if self._use_log_odds:
            new_lo = [0.0] * new_n
        else:
            new_bin = [OCC_UNKNOWN] * new_n
        new_obs = [False] * new_n
        for r in range(self._height):
            src_row = r * self._width
            dst_row = (r + shift_r) * new_w + shift_c
            for c in range(self._width):
                si = src_row + c
                di = dst_row + c
                if self._use_log_odds:
                    new_lo[di] = self._log_odds[si]
                else:
                    new_bin[di] = self._binary[si]
                new_obs[di] = self._observed[si]

        if self._use_log_odds:
            self._log_odds = new_lo
        else:
            self._binary = new_bin
        self._observed = new_obs
        self._origin_x -= shift_c * self._resolution
        self._origin_y -= shift_r * self._resolution
        self._width = new_w
        self._height = new_h
        self._growth_count += 1

        self.get_logger().info(
            f'Grid grew to {self._width}x{self._height}, '
            f'origin=({self._origin_x:.1f},{self._origin_y:.1f}), '
            f'total growths={self._growth_count}'
        )
        return shift_c, shift_r

    def _update_free(self, col, row):
        if not self._in_bounds(col, row):
            return
        i = self._idx(col, row)
        self._observed[i] = True
        if self._use_log_odds:
            self._log_odds[i] = max(self._l_min, self._log_odds[i] + self._l_free)
        else:
            self._binary[i] = OCC_FREE

    def _update_occ(self, col, row):
        if not self._in_bounds(col, row):
            return
        i = self._idx(col, row)
        self._observed[i] = True
        if self._use_log_odds:
            self._log_odds[i] = min(self._l_max, self._log_odds[i] + self._l_occ)
        else:
            self._binary[i] = OCC_OCCUPIED

    def _scan_callback(self, msg):
        if self._latest_odom is None:
            return
        rx = self._latest_odom.pose.pose.position.x
        ry = self._latest_odom.pose.pose.position.y
        ryaw = yaw_from_quaternion(self._latest_odom.pose.pose.orientation)

        stride = max(1, int(self.get_parameter('scan_stride').value))
        treat_max = bool(self.get_parameter('treat_max_range_as_free').value)

        # First pass: compute endpoints and bbox so we can grow the grid once.
        endpoints = []
        min_c = max_c = None
        min_r = max_r = None
        rc, rr = self._world_to_grid(rx, ry)
        min_c, max_c, min_r, max_r = rc, rc, rr, rr
        for i in range(0, len(msg.ranges), stride):
            r = msg.ranges[i]
            if math.isnan(r) or math.isinf(r):
                continue
            if r < msg.range_min:
                continue
            is_max = r >= msg.range_max
            if is_max and not treat_max:
                continue
            r_eff = min(r, msg.range_max)
            angle = msg.angle_min + i * msg.angle_increment
            ex = rx + r_eff * math.cos(ryaw + angle)
            ey = ry + r_eff * math.sin(ryaw + angle)
            ec, er = self._world_to_grid(ex, ey)
            endpoints.append((ec, er, is_max))
            if ec < min_c: min_c = ec
            if ec > max_c: max_c = ec
            if er < min_r: min_r = er
            if er > max_r: max_r = er

        # Grow the grid in one shot if any endpoint fell outside.
        shift_c, shift_r = self._grow_if_needed(min_c, min_r, max_c, max_r)
        rc += shift_c
        rr += shift_r

        # Second pass: raycast and update each beam.
        for ec, er, is_max in endpoints:
            ec += shift_c
            er += shift_r
            for cx, cy in bresenham(rc, rr, ec, er):
                if (cx, cy) == (ec, er):
                    if is_max:
                        self._update_free(cx, cy)
                    else:
                        self._update_occ(cx, cy)
                else:
                    self._update_free(cx, cy)

        self._scans_processed += 1

    def _publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = self._frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self._resolution
        msg.info.width = self._width
        msg.info.height = self._height
        msg.info.origin.position.x = self._origin_x
        msg.info.origin.position.y = self._origin_y
        msg.info.origin.orientation.w = 1.0

        n = self._width * self._height
        if self._use_log_odds:
            data = [OCC_FREE] * n
            for i in range(n):
                if not self._observed[i]:
                    data[i] = OCC_UNKNOWN
                elif self._log_odds[i] > self._l_th_occ:
                    data[i] = OCC_OCCUPIED
                elif self._log_odds[i] < self._l_th_free:
                    data[i] = OCC_FREE
                else:
                    # Map intermediate log-odds linearly to 0..100.
                    p = 1.0 / (1.0 + math.exp(-self._log_odds[i]))
                    data[i] = int(round(p * OCC_OCCUPIED))
            msg.data = data
        else:
            msg.data = list(self._binary)
        self._map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = OccupancyMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'Ctrl+C received. Final grid {node._width}x{node._height} '
            f'after {node._growth_count} growths, {node._scans_processed} scans.'
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
