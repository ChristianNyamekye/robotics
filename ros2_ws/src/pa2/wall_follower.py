#!/usr/bin/env python

# Author: Christian Nyamekye
# Date: 2026-04-28
# Course: COSC 81/281 - Principles of Robot Design and Programming
# Assignment: PA2 - The Wall-Following Robot

"""
Right-wall-following ROS 2 node for the Stage simulator.

Subscribes to /base_scan, publishes /cmd_vel. A discrete PID on the
predicted lateral distance to the right-hand wall produces angular
velocity; an FSM handles internal corners and lost-wall recovery.
"""

import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan'  # Stage; override to 'scan' for Gazebo
CONTROL_RATE = 20.0  # Hz

DEFAULT_LINEAR_VELOCITY = 0.5
DEFAULT_TARGET_DISTANCE = 0.5
DEFAULT_KP = 2.0
DEFAULT_KI = 0.0
DEFAULT_KD = 0.3
DEFAULT_LOOK_AHEAD = 0.6
DEFAULT_BEAM_THETA_DEG = 60.0
DEFAULT_MAX_ANGULAR = 1.5
DEFAULT_FRONT_THRESHOLD = 0.7
DEFAULT_LOST_WALL_THRESHOLD = 2.0
DEFAULT_FRONT_FOV_DEG = 20.0
DEFAULT_RIGHT_BEAM_DEG = -90.0
DEFAULT_INTEGRAL_LIMIT = 1.0
DEFAULT_RECOVER_LINEAR_FRACTION = 0.5
DEFAULT_SEARCH_OMEGA_FRACTION = 0.5

DEFAULT_ADAPTIVE_VELOCITY = True
DEFAULT_ADAPTIVE_MIN_FRACTION = 0.4
DEFAULT_ADAPTIVE_ERROR_SCALE = 2.0

USE_SIM_TIME = True


class FsmState(Enum):
    INIT = 0
    FOLLOW = 1
    CORNER = 2
    SEARCH = 3


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, USE_SIM_TIME,
        )])

        self.declare_parameter('kp', DEFAULT_KP)
        self.declare_parameter('ki', DEFAULT_KI)
        self.declare_parameter('kd', DEFAULT_KD)
        self.declare_parameter('linear_velocity', DEFAULT_LINEAR_VELOCITY)
        self.declare_parameter('target_distance', DEFAULT_TARGET_DISTANCE)
        self.declare_parameter('look_ahead', DEFAULT_LOOK_AHEAD)
        self.declare_parameter('beam_theta_deg', DEFAULT_BEAM_THETA_DEG)
        self.declare_parameter('max_angular_velocity', DEFAULT_MAX_ANGULAR)
        self.declare_parameter('front_threshold', DEFAULT_FRONT_THRESHOLD)
        self.declare_parameter('lost_wall_threshold', DEFAULT_LOST_WALL_THRESHOLD)
        self.declare_parameter('front_fov_deg', DEFAULT_FRONT_FOV_DEG)
        self.declare_parameter('right_beam_deg', DEFAULT_RIGHT_BEAM_DEG)
        self.declare_parameter('integral_limit', DEFAULT_INTEGRAL_LIMIT)
        self.declare_parameter('recover_linear_fraction', DEFAULT_RECOVER_LINEAR_FRACTION)
        self.declare_parameter('search_omega_fraction', DEFAULT_SEARCH_OMEGA_FRACTION)
        self.declare_parameter('scan_topic', DEFAULT_SCAN_TOPIC)
        self.declare_parameter('adaptive_velocity', DEFAULT_ADAPTIVE_VELOCITY)
        self.declare_parameter('adaptive_min_fraction', DEFAULT_ADAPTIVE_MIN_FRACTION)
        self.declare_parameter('adaptive_error_scale', DEFAULT_ADAPTIVE_ERROR_SCALE)

        scan_topic = self.get_parameter('scan_topic').value
        self._cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, 10
        )
        self.get_logger().info(f'Subscribed to scan topic: {scan_topic}')

        self._scan = None
        self._fsm = FsmState.INIT
        self._error_prev = 0.0
        self._error_integral = 0.0
        self._last_time = None

        self._timer = self.create_timer(1.0 / CONTROL_RATE, self._control_loop)

    def _scan_callback(self, msg):
        self._scan = msg

    @staticmethod
    def _valid_range(scan, idx):
        n = len(scan.ranges)
        if not 0 <= idx < n:
            return float('inf')
        r = scan.ranges[idx]
        if math.isnan(r) or math.isinf(r) or r < scan.range_min or r > scan.range_max:
            return float('inf')
        return r

    def _angle_to_index(self, scan, angle_rad):
        idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
        return max(0, min(len(scan.ranges) - 1, idx))

    def _min_in_window(self, scan, center_rad, half_width_rad):
        i_lo = self._angle_to_index(scan, center_rad - half_width_rad)
        i_hi = self._angle_to_index(scan, center_rad + half_width_rad)
        m = float('inf')
        for i in range(i_lo, i_hi + 1):
            r = self._valid_range(scan, i)
            if r < m:
                m = r
        return m

    def _measure_wall(self, scan):
        """
        Predictive lateral distance from two right-side beams.

        b = perpendicular beam (-90), a = forward-diagonal beam (-90 + theta).
        alpha = atan2(a*cos(theta) - b, a*sin(theta)) is the wall's angle
        relative to the robot's heading; D_t = b*cos(alpha) is the current
        perpendicular distance and D_t+1 = D_t + L*sin(alpha) projects it
        forward by the look-ahead distance L. Feeding D_t+1 to the PID lets
        the controller anticipate curvature without raising K_d.
        """
        right_perp = math.radians(self.get_parameter('right_beam_deg').value)
        theta = math.radians(self.get_parameter('beam_theta_deg').value)
        L = self.get_parameter('look_ahead').value

        b = self._valid_range(scan, self._angle_to_index(scan, right_perp))
        a = self._valid_range(scan, self._angle_to_index(scan, right_perp + theta))

        if math.isinf(a) or math.isinf(b):
            return float('inf'), float('inf'), 0.0, b

        alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        d_now = b * math.cos(alpha)
        d_future = d_now + L * math.sin(alpha)
        return d_now, d_future, alpha, b

    def _control_loop(self):
        if self._scan is None:
            return

        scan = self._scan

        if self._fsm == FsmState.INIT:
            self._fsm = FsmState.FOLLOW
            self.get_logger().info('Wall follower active.')

        d_now, d_future, alpha, b = self._measure_wall(scan)
        front_min = self._min_in_window(
            scan, 0.0, math.radians(self.get_parameter('front_fov_deg').value),
        )

        front_thresh = self.get_parameter('front_threshold').value
        lost_thresh = self.get_parameter('lost_wall_threshold').value

        if front_min < front_thresh:
            new_state = FsmState.CORNER
        elif b > lost_thresh:
            new_state = FsmState.SEARCH
        else:
            new_state = FsmState.FOLLOW

        if new_state != self._fsm:
            self.get_logger().info(
                f'FSM: {self._fsm.name} -> {new_state.name} '
                f'(front={front_min:.2f}, right={b:.2f}, d_future={d_future:.2f})'
            )
            # Recovery states must not inherit accumulated PID memory.
            if new_state != FsmState.FOLLOW:
                self._reset_pid()
            self._fsm = new_state

        v_target = self.get_parameter('linear_velocity').value
        omega_max = self.get_parameter('max_angular_velocity').value
        recover_frac = self.get_parameter('recover_linear_fraction').value
        search_omega = self.get_parameter('search_omega_fraction').value

        if self._fsm == FsmState.FOLLOW:
            v_cmd, omega_cmd = self._follow_command(d_future, v_target, omega_max)
        elif self._fsm == FsmState.CORNER:
            v_cmd = recover_frac * v_target
            omega_cmd = +omega_max
        elif self._fsm == FsmState.SEARCH:
            v_cmd = recover_frac * v_target
            omega_cmd = -search_omega * omega_max
        else:
            v_cmd, omega_cmd = 0.0, 0.0

        twist = Twist()
        twist.linear.x = v_cmd
        twist.angular.z = omega_cmd
        self._cmd_pub.publish(twist)

    def _follow_command(self, distance_measured, v_target, omega_max):
        """Discrete PID -> angular velocity. Positive omega = turn left (away from right wall)."""
        target = self.get_parameter('target_distance').value
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        i_limit = self.get_parameter('integral_limit').value

        now = time.monotonic()
        dt = 1.0 / CONTROL_RATE if self._last_time is None else max(1e-3, now - self._last_time)
        self._last_time = now

        error = target - distance_measured

        self._error_integral += error * dt
        self._error_integral = max(-i_limit, min(i_limit, self._error_integral))

        d_error = (error - self._error_prev) / dt
        self._error_prev = error

        omega = kp * error + ki * self._error_integral + kd * d_error
        omega = max(-omega_max, min(omega_max, omega))

        # Adaptive velocity (extra credit): linearly scale forward speed down
        # as |error| grows so the robot slows on bends and speeds up on
        # straights. Floor at adaptive_min_fraction so we never stall.
        if self.get_parameter('adaptive_velocity').value:
            min_frac = self.get_parameter('adaptive_min_fraction').value
            scale = self.get_parameter('adaptive_error_scale').value
            v_scale = max(min_frac, 1.0 - scale * abs(error))
            v_target = v_target * v_scale

        return v_target, omega

    def _reset_pid(self):
        self._error_prev = 0.0
        self._error_integral = 0.0
        self._last_time = None

    def stop(self):
        self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WallFollower()
    try:
        rclpy.spin(node)
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
