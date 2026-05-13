#!/usr/bin/env python

# Author: Christian Nyamekye
# Date: 2026-04-18
# Course: COSC 81/281 - Principles of Robot Design and Programming
# Assignment: PA1 - The Shape-Drawing Robot

"""
ROS 2 node that drives a TurtleBot3 along a trapezoid, D-shape, or
user-supplied polygon using the differential-drive kinematic model.
Open-loop (time-based) and closed-loop (odom-feedback, extra credit).
"""

import math
import sys
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_ODOM_TOPIC = 'odom'
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_footprint'

FREQUENCY = 20  # Hz
WHEEL_BASE = 0.160  # m (TurtleBot3 Burger)

LINEAR_VELOCITY = 0.10            # m/s
ANGULAR_VELOCITY = math.pi / 6    # rad/s

POSITION_TOLERANCE = 0.02  # m
HEADING_TOLERANCE  = 0.02  # rad
KP_LINEAR  = 0.8
KP_ANGULAR = 1.5

USE_SIM_TIME = True
STARTUP_TIMEOUT = 15.0  # s
STRAIGHT_LINE_EPS = 1e-6


class fsm(Enum):
    IDLE = 0
    EXECUTING = 1
    DONE = 2


def normalize_angle(theta):
    """Wrap an angle into (-pi, pi]."""
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta <= -math.pi:
        theta += 2.0 * math.pi
    return theta


def yaw_from_quaternion(q):
    """Yaw from a geometry_msgs Quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class ShapeDrawer(Node):
    def __init__(self,
                 linear_velocity=LINEAR_VELOCITY,
                 angular_velocity=ANGULAR_VELOCITY,
                 wheel_base=WHEEL_BASE,
                 node_name="shape_drawer",
                 context=None):
        super().__init__(node_name, context=context)

        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, USE_SIM_TIME,
        )])

        self._cmd_pub = self.create_publisher(Twist, DEFAULT_CMD_VEL_TOPIC, 10)
        self._odom_sub = self.create_subscription(
            Odometry, DEFAULT_ODOM_TOPIC, self._odom_callback, 10
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.wheel_base = wheel_base

        self._current_x = 0.0
        self._current_y = 0.0
        self._current_yaw = 0.0
        self._odom_received = False

        self._fsm = fsm.IDLE

    def _odom_callback(self, msg):
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        self._current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self._odom_received = True

    def _wait_for_sim_ready(self, timeout_sec):
        """Block until sim clock, cmd_vel subscriber, and odom are all ready."""
        self.get_logger().info('Waiting for simulation to be ready...')
        start_time = time.monotonic()
        clock_ready = not USE_SIM_TIME

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = self.get_clock().now()
            if time.monotonic() - start_time >= timeout_sec:
                self.get_logger().warn('Startup wait timeout reached. Continuing anyway.')
                return

            if USE_SIM_TIME and now.nanoseconds > 0:
                clock_ready = True

            cmd_ready = self._cmd_pub.get_subscription_count() > 0
            if clock_ready and cmd_ready and self._odom_received:
                self.get_logger().info('Simulation ready. Node ready for shape commands.')
                return

    def move(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = float(linear_vel)
        twist.angular.z = float(angular_vel)
        self._cmd_pub.publish(twist)

    def stop(self):
        self._cmd_pub.publish(Twist())

    def _sim_sleep(self, seconds):
        end = self.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def _drive_for_duration(self, linear_vel, angular_vel, duration_sec):
        """Publish (linear, angular) at FREQUENCY Hz for duration_sec sim seconds."""
        # Drain /clock so end_time is computed from fresh sim time. A long
        # idle (input() prompts, tf lookups) otherwise leaves cached sim time
        # stale; the first spin_once inside the loop then jumps time forward
        # by the idle interval and the loop exits without publishing any Twist.
        pump_deadline = time.monotonic() + 0.05
        while time.monotonic() < pump_deadline:
            rclpy.spin_once(self, timeout_sec=0.005)

        end_time = self.get_clock().now() + Duration(seconds=duration_sec)
        dt = 1.0 / FREQUENCY
        while rclpy.ok() and self.get_clock().now() < end_time:
            self.move(linear_vel, angular_vel)
            self._sim_sleep(dt)
        self.stop()

    def rotate_in_place(self, delta_theta):
        """Rotate by delta_theta (+ CCW, - CW). Uses vl = -vr."""
        if abs(delta_theta) < STRAIGHT_LINE_EPS:
            return
        angular = math.copysign(self.angular_velocity, delta_theta)
        duration = abs(delta_theta) / self.angular_velocity
        self._drive_for_duration(0.0, angular, duration)

    def drive_straight(self, distance):
        """Drive forward by distance meters (vl = vr). The R -> infinity case."""
        if abs(distance) < STRAIGHT_LINE_EPS:
            return
        linear = math.copysign(self.linear_velocity, distance)
        duration = abs(distance) / self.linear_velocity
        self._drive_for_duration(linear, 0.0, duration)

    def drive_arc(self, radius, delta_theta):
        """
        Arc along a circle of radius `radius`, sweeping `delta_theta` (+ CCW).
        ICC model: |omega| = v/R, duration = |delta_theta|*R/v,
                   v_r = v + omega*L/2, v_l = v - omega*L/2.
        """
        if radius <= STRAIGHT_LINE_EPS:
            self.rotate_in_place(delta_theta)
            return
        v = self.linear_velocity
        omega = math.copysign(v / radius, delta_theta)
        duration = abs(delta_theta) * radius / v

        v_r = v + omega * self.wheel_base / 2.0
        v_l = v - omega * self.wheel_base / 2.0
        self.get_logger().info(
            f'Arc: R={radius:.3f}, dtheta={delta_theta:.3f}, '
            f'v={v:.3f}, omega={omega:.3f}, vl={v_l:.3f}, vr={v_r:.3f}, T={duration:.3f}'
        )
        self._drive_for_duration(v, omega, duration)

    def rotate_in_place_closed(self, delta_theta):
        """Rotate to current_yaw + delta_theta using odom + P control."""
        if abs(delta_theta) < STRAIGHT_LINE_EPS:
            return
        rclpy.spin_once(self, timeout_sec=0.05)
        target_yaw = normalize_angle(self._current_yaw + delta_theta)
        dt = 1.0 / FREQUENCY

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            err = normalize_angle(target_yaw - self._current_yaw)
            if abs(err) < HEADING_TOLERANCE:
                break
            cmd = KP_ANGULAR * err
            if cmd > self.angular_velocity:
                cmd = self.angular_velocity
            elif cmd < -self.angular_velocity:
                cmd = -self.angular_velocity
            self.move(0.0, cmd)
            self._sim_sleep(dt)
        self.stop()

    def drive_to_point_closed(self, goal_x, goal_y):
        """Drive to odom-frame (goal_x, goal_y) using odom + P control."""
        rclpy.spin_once(self, timeout_sec=0.05)
        dx = goal_x - self._current_x
        dy = goal_y - self._current_y
        if math.hypot(dx, dy) > POSITION_TOLERANCE:
            target_heading = math.atan2(dy, dx)
            self.rotate_in_place_closed(
                normalize_angle(target_heading - self._current_yaw)
            )

        dt = 1.0 / FREQUENCY
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            dx = goal_x - self._current_x
            dy = goal_y - self._current_y
            dist = math.hypot(dx, dy)
            if dist < POSITION_TOLERANCE:
                break
            target_heading = math.atan2(dy, dx)
            heading_err = normalize_angle(target_heading - self._current_yaw)

            linear_cmd = min(KP_LINEAR * dist, self.linear_velocity)
            angular_cmd = KP_ANGULAR * heading_err
            if angular_cmd > self.angular_velocity:
                angular_cmd = self.angular_velocity
            elif angular_cmd < -self.angular_velocity:
                angular_cmd = -self.angular_velocity

            # Stop advancing if we're badly off-heading; rotate back on target first.
            if abs(heading_err) > math.pi / 6:
                linear_cmd = 0.0

            self.move(linear_cmd, angular_cmd)
            self._sim_sleep(dt)
        self.stop()

    def draw_trapezoid(self, r, closed_loop=False):
        """
        Isosceles "simplified-D" trapezoid of radius r. Vertices
        B1=(0,r), V1=(r cos45, r sin45), V2=(r cos-45, r sin-45), B2=(0,-r).
        Robot starts at midpoint of longer base; base is traced in two halves.
        Path: start -> B1 -> V1 -> V2 -> B2 -> start (CW).
        """
        self._fsm = fsm.EXECUTING
        self.get_logger().info(f'Drawing TRAPEZOID (r = {r:.3f} m, closed_loop={closed_loop})')

        leg_len = r * math.sqrt(2.0 - math.sqrt(2.0))  # |B1 - V1|

        if closed_loop:
            rclpy.spin_once(self, timeout_sec=0.05)
            x0, y0, th0 = self._current_x, self._current_y, self._current_yaw
            B1_x = x0 + r * math.cos(th0 + math.pi / 2.0)
            B1_y = y0 + r * math.sin(th0 + math.pi / 2.0)
            V1_x = x0 + r * math.cos(th0 + math.pi / 4.0)
            V1_y = y0 + r * math.sin(th0 + math.pi / 4.0)
            V2_x = x0 + r * math.cos(th0 - math.pi / 4.0)
            V2_y = y0 + r * math.sin(th0 - math.pi / 4.0)
            B2_x = x0 + r * math.cos(th0 - math.pi / 2.0)
            B2_y = y0 + r * math.sin(th0 - math.pi / 2.0)
            self.drive_to_point_closed(B1_x, B1_y)
            self.drive_to_point_closed(V1_x, V1_y)
            self.drive_to_point_closed(V2_x, V2_y)
            self.drive_to_point_closed(B2_x, B2_y)
            self.drive_to_point_closed(x0, y0)
            self.rotate_in_place_closed(normalize_angle(th0 - self._current_yaw))
        else:
            self.rotate_in_place(math.pi / 2.0)
            self.drive_straight(r)
            self.rotate_in_place(-5.0 * math.pi / 8.0)
            self.drive_straight(leg_len)
            self.rotate_in_place(-3.0 * math.pi / 8.0)
            self.drive_straight(r * math.sqrt(2.0))
            self.rotate_in_place(-3.0 * math.pi / 8.0)
            self.drive_straight(leg_len)
            self.rotate_in_place(-5.0 * math.pi / 8.0)
            self.drive_straight(r)
            self.rotate_in_place(-math.pi / 2.0)

        self._fsm = fsm.DONE
        self.get_logger().info('Trapezoid complete.')

    def draw_d_shape(self, r, closed_loop=False):
        """
        D-shape: vertical stroke of length 2r (split by the start midpoint) +
        semicircle of radius r centered at start, bulging along local +x.
        Sequence: rot +pi/2, drive r, rot -pi/2, arc(r, -pi), rot -pi/2,
        drive r, rot -pi/2 (restore heading).
        """
        self._fsm = fsm.EXECUTING
        self.get_logger().info(f'Drawing D-SHAPE (r = {r:.3f} m, closed_loop={closed_loop})')

        if closed_loop:
            rclpy.spin_once(self, timeout_sec=0.05)
            x0, y0, th0 = self._current_x, self._current_y, self._current_yaw
            P_top_x = x0 + r * math.cos(th0 + math.pi / 2.0)
            P_top_y = y0 + r * math.sin(th0 + math.pi / 2.0)
            P_bot_x = x0 + r * math.cos(th0 - math.pi / 2.0)
            P_bot_y = y0 + r * math.sin(th0 - math.pi / 2.0)

            self.drive_to_point_closed(P_top_x, P_top_y)
            self.rotate_in_place_closed(normalize_angle(th0 - self._current_yaw))
            self.drive_arc(r, -math.pi)
            self.rotate_in_place_closed(
                normalize_angle((th0 + math.pi / 2.0) - self._current_yaw)
            )
            self.drive_to_point_closed(x0, y0)
            self.rotate_in_place_closed(normalize_angle(th0 - self._current_yaw))
        else:
            self.rotate_in_place(math.pi / 2.0)
            self.drive_straight(r)
            self.rotate_in_place(-math.pi / 2.0)
            self.drive_arc(r, -math.pi)
            self.rotate_in_place(-math.pi / 2.0)
            self.drive_straight(r)
            self.rotate_in_place(-math.pi / 2.0)

        self._fsm = fsm.DONE
        self.get_logger().info('D-shape complete.')

    def draw_polygon(self, vertices, closed_loop=False):
        """
        Visit each odom-frame vertex in order then return to the first. Each
        open-loop leg re-reads the current pose via tf2, so drift between
        legs is re-observed rather than compounded.
        """
        self._fsm = fsm.EXECUTING
        self.get_logger().info(
            f'Drawing POLYGON with {len(vertices)} vertices (closed_loop={closed_loop})'
        )
        if len(vertices) < 2:
            self.get_logger().warn('Polygon needs at least 2 vertices.')
            self._fsm = fsm.DONE
            return

        waypoints = list(vertices) + [vertices[0]]

        for idx, (gx, gy) in enumerate(waypoints):
            self.get_logger().info(f'  -> vertex {idx}/{len(waypoints)-1}: ({gx:.3f}, {gy:.3f})')
            if closed_loop:
                self.drive_to_point_closed(gx, gy)
            else:
                self._drive_to_point_open(gx, gy)

        self._fsm = fsm.DONE
        self.get_logger().info('Polygon complete.')

    def _drive_to_point_open(self, goal_x, goal_y):
        """Open-loop leg: lookup pose, rotate toward goal, drive distance."""
        try:
            trans = self._tf_buffer.lookup_transform(
                ODOM_FRAME, BASE_FRAME, Time(),
                timeout=Duration(seconds=1.0),
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'tf lookup failed: {ex}')
            return

        rx = trans.transform.translation.x
        ry = trans.transform.translation.y
        ryaw = yaw_from_quaternion(trans.transform.rotation)

        dx = goal_x - rx
        dy = goal_y - ry
        distance = math.hypot(dx, dy)
        if distance < POSITION_TOLERANCE:
            return

        target_heading = math.atan2(dy, dx)
        delta = normalize_angle(target_heading - ryaw)

        self.get_logger().info(
            f'    tf: robot=({rx:.3f},{ry:.3f},{ryaw:.3f}) goal=({goal_x:.3f},{goal_y:.3f}) '
            f'rot={delta:.3f} rad, dist={distance:.3f} m'
        )
        self.rotate_in_place(delta)
        self.drive_straight(distance)

    def run_shape_menu(self):
        """Prompt user for shape + parameters, then dispatch."""
        self._wait_for_sim_ready(STARTUP_TIMEOUT)
        # spin_once after each input() keeps the middleware alive through blocking prompts.
        rclpy.spin_once(self, timeout_sec=0.05)

        print('\n============================================')
        print('  PA1 - Shape-Drawing Robot')
        print('  1) Trapezoid')
        print('  2) D-shape')
        print('  3) Polygon (vertex list in odom frame)')
        choice = input('Select shape [1-3]: ').strip()
        rclpy.spin_once(self, timeout_sec=0.05)

        closed_str = input('Use closed-loop control (extra credit)? [y/N]: ').strip().lower()
        closed_loop = (closed_str == 'y')
        rclpy.spin_once(self, timeout_sec=0.05)

        if choice == '1':
            r = float(input('Enter radius r (m): '))
            rclpy.spin_once(self, timeout_sec=0.05)
            self.draw_trapezoid(r, closed_loop=closed_loop)
        elif choice == '2':
            r = float(input('Enter radius r (m): '))
            rclpy.spin_once(self, timeout_sec=0.05)
            self.draw_d_shape(r, closed_loop=closed_loop)
        elif choice == '3':
            print('Enter vertices as space-separated "x,y" pairs in the odom frame.')
            print('  Example:  1.0,0.0  1.0,1.0  0.0,1.0')
            raw = input('Vertices: ').strip()
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                vertices = [tuple(map(float, token.split(','))) for token in raw.split()]
            except ValueError:
                print('Invalid input. Expected "x,y x,y ..."')
                return
            self.draw_polygon(vertices, closed_loop=closed_loop)
        else:
            print('Invalid choice.')
            return


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    shape_drawer = ShapeDrawer()
    try:
        shape_drawer.run_shape_menu()
    except KeyboardInterrupt:
        shape_drawer.get_logger().error("ROS node interrupted.")
    finally:
        if rclpy.ok():
            shape_drawer.stop()
        shape_drawer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
