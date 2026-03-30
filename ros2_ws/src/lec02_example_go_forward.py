#!/usr/bin/env python3

# Author: Alberto Quattrini Li
# Date: 2026-03-30
# Description: Example node to move forward for a fixed duration, with laser scan subscription.
# Acknowledgments: Code formatting and comment cleanup assisted by GitHub Copilot.

# Import of relevant libraries.
import rclpy # module for ROS APIs
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from rclpy.duration import Duration
from geometry_msgs.msg import Twist # message type
from sensor_msgs.msg import LaserScan # message type

import time

# Constants.
FREQUENCY = 10 #Hz.
LINEAR_VELOCITY = 0.125 #m/s
DURATION = 8.0 #s how long the message should be published.
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan'
USE_SIM_TIME = True
STARTUP_TIMEOUT = 15.0 #s. Max wait for simulator/controller startup.

class GoForward(Node):
    def __init__(self, linear_velocity=LINEAR_VELOCITY,
                 node_name="go_forward", context=None):
        """Constructor."""
        super().__init__(node_name, context=context)

        # Workaround not to use roslaunch
        use_sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            USE_SIM_TIME
        )
        self.set_parameters([use_sim_time_param])

        # Set up publisher.
        self._cmd_pub = self.create_publisher(Twist, DEFAULT_CMD_VEL_TOPIC, 1)
        # Set up laser subscriber.
        self._scan_sub = self.create_subscription(
            LaserScan,
            DEFAULT_SCAN_TOPIC,
            self._scan_callback,
            10,
        )

        # Other variables.
        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.latest_scan = None

    def move_forward(self, duration):
        """Function to move_forward for a given duration."""
        if duration <= 0.0:
            self.get_logger().warn('Duration must be > 0. Nothing to do.')
            return

        self._wait_for_sim_ready(STARTUP_TIMEOUT)

        self.get_logger().info('Starting forward motion...')

        duration = Duration(seconds=duration)
        rclpy.spin_once(self)
        start_time = self.get_clock().now()

        # Loop.
        while rclpy.ok():
            # Process subscriptions while pacing this control loop.
            rclpy.spin_once(self, timeout_sec=1.0 / FREQUENCY)
            # Check if traveled of given distance based on time.
            if self.get_clock().now() - start_time >= duration:
                break

            # Publish message.
            self._publish_velocity(self.linear_velocity, 0.0)

        # Traveled the required distance, stop.
        self.stop()
        self.get_logger().info('Motion completed.')

    def _wait_for_sim_ready(self, timeout_sec):
        """Wait until simulation clock and cmd_vel subscriber are ready."""
        self.get_logger().info('Waiting for simulation to be ready...')

        start_time = time.monotonic()

        clock_ready = not USE_SIM_TIME
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            now = self.get_clock().now()
            if time.monotonic() - start_time >= timeout_sec:
                self.get_logger().warn('Startup wait timeout reached. Continuing anyway.')
                return

            # With sim time, clock starts advancing only when /clock is available.
            if USE_SIM_TIME and now.nanoseconds > 0:
                clock_ready = True

            cmd_ready = self._cmd_pub.get_subscription_count() > 0
            if clock_ready and cmd_ready:
                self.get_logger().info('Simulation ready. Starting motion.')
                return

    def stop(self):
        """Stop the robot."""
        self._publish_velocity(0.0, 0.0)

    def _publish_velocity(self, linear_x, angular_z):
        """Publish a velocity command."""
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self._cmd_pub.publish(twist_msg)

    def stop_and_flush(self, repeats=3, timeout_sec=0.05):
        """Publish stop commands and briefly spin to increase delivery reliability."""
        for _ in range(repeats):
            self.stop()
            rclpy.spin_once(self, timeout_sec=timeout_sec)

    def _scan_callback(self, msg):
        """Store latest scan for later use in other methods."""
        self.latest_scan = msg

    def get_front_distance(self):
        """Example helper: return distance at index 0 if scan is available."""
        if self.latest_scan is None or not self.latest_scan.ranges:
            return None
        return self.latest_scan.ranges[0]

def main(args=None):
    # 1st. initialization of node.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    go_forward = GoForward()
    try:
        go_forward.move_forward(DURATION)
    except KeyboardInterrupt:
        go_forward.get_logger().info('Ctrl+C received. Sending stop command...')
        go_forward.stop_and_flush()
    finally:
        # Ensure a final zero-velocity command is attempted before teardown.
        if rclpy.ok():
            go_forward.stop_and_flush()
        go_forward.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    

if __name__ == "__main__":
    main()