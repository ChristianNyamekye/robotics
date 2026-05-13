#!/usr/bin/env python3
# The line above is important so that this file is interpreted with Python when running it.

# Author: TODO: Christian Nyamekye
# Date: TODO: 6th April 2026

# Import of python modules.
import math # use of pi.
import random # use for generating a random real number
import time

# import of relevant libraries.
import rclpy # module for ROS APIs
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan

# NOTE: there might be some other libraries that can be useful
# as seen in lec02_example_go_forward.py, e.g., Duration

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan' # name of topic for Stage simulator. For Gazebo, 'scan'

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# Velocities that will be used (TODO: feel free to tune)
LINEAR_VELOCITY = 0.2 # m/s
ANGULAR_VELOCITY = math.pi / 4 # rad/s

# Random turn angle range in degrees (TODO: feel free to tune)
RANDOM_TURN_MIN_DEG = -180.0 # deg
RANDOM_TURN_MAX_DEG = 180.0 # deg

# Threshold of minimum clearance distance (TODO: feel free to tune)
MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, should be smaller than range_max

# Field of view in radians that is checked in front of the robot (TODO: feel free to tune)
MIN_SCAN_ANGLE_RAD = -10.0 / 180.0 * math.pi
MAX_SCAN_ANGLE_RAD = +10.0 / 180.0 * math.pi

USE_SIM_TIME = True
STARTUP_TIMEOUT = 15.0 # s. Max wait for simulator/controller startup.


class RandomWalk(Node):
    def __init__(
        self,
        linear_velocity=LINEAR_VELOCITY,
        angular_velocity=ANGULAR_VELOCITY,
        min_threshold_distance=MIN_THRESHOLD_DISTANCE,
        random_turn_min_deg=RANDOM_TURN_MIN_DEG,
        random_turn_max_deg=RANDOM_TURN_MAX_DEG,
        scan_angle=None,
        node_name='random_walk',
        context=None,
    ):
        """Constructor."""
        super().__init__(node_name, context=context)

        if scan_angle is None:
            scan_angle = (MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD)

        # Workaround not to use roslaunch
        use_sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            USE_SIM_TIME,
        )
        self.set_parameters([use_sim_time_param])

        # Setting up publishers/subscribers.
        self._cmd_pub = self.create_publisher(Twist, DEFAULT_CMD_VEL_TOPIC, 1)
        self._laser_sub = self.create_subscription(
            LaserScan,
            DEFAULT_SCAN_TOPIC,
            self._laser_callback,
            1,
        )

        # Parameters.
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.min_threshold_distance = min_threshold_distance
        self.random_turn_min_deg = random_turn_min_deg
        self.random_turn_max_deg = random_turn_max_deg
        self.scan_angle = tuple(scan_angle)

        # Flag used to control the behavior of the robot.
        self._close_obstacle = False
        self._control_timer = None

        # State variables for rotation
        self._rotation_target_time= None
        self._rotation_angular_velocity = 0.0

        # Dynamic recovery: re-check FOV after a rotation before resuming forward.
        self._verifying_clearance = False
        self._latest_front_distance = None
    

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

            if USE_SIM_TIME and now.nanoseconds > 0:
                clock_ready = True

            cmd_ready = self._cmd_pub.get_subscription_count() > 0
            if clock_ready and cmd_ready:
                self.get_logger().info('Simulation ready. Starting random walk.')
                return

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def stop(self):
        """Stop the robot."""
        self.move(0.0, 0.0)

    def stop_and_flush(self, repeats=3, timeout_sec=0.05):
        """Publish stop commands and briefly spin to increase delivery reliability."""
        if not rclpy.ok():
            return
        for _ in range(repeats):
            self.stop()
            rclpy.spin_once(self, timeout_sec=timeout_sec)

    def _closest_obstacle_in_front(self, msg):
        """Return the minimum valid range within the configured forward FOV."""
        # convert the scan_angle to indices in ranges[] array
        min_index = int(math.floor((self.scan_angle[0] - msg.angle_min) / msg.angle_increment))
        max_index = int(math.ceil((self.scan_angle[1] - msg.angle_min) / msg.angle_increment))

        min_index = max(0, min_index)
        max_index = min(len(msg.ranges) - 1, max_index)

        # find the minimum range value between min_index and max_index
        min_range = float('inf')
        for i in range(min_index, max_index + 1):
            range_value = msg.ranges[i]
            if (msg.range_min <= range_value <= msg.range_max and not math.isnan(range_value) and not
            math.isinf(range_value)):
                if range_value < min_range:
                    min_range = range_value
        return min_range

    def _laser_callback(self, msg):
        """Processing of laser message."""
        # Access to the index of the measurement in front of the robot.
        # NOTE: index 0 corresponds to min_angle,
        #       index 1 corresponds to min_angle + angle_inc
        #       index 2 corresponds to min_angle + angle_inc * 2
        #       ...

        if not self._close_obstacle:
            # Find the minimum range value between min_scan_angle and
            # max_scan_angle
            # If the minimum range value found is closer to min_threshold_distance, change the flag self._close_obstacle
            # Note: You have to find the min index and max index.
            # Please double check the LaserScan message http://docs.ros.org/en/humble/p/sensor_msgs/msg/LaserScan.html
            ####### TODO: ANSWER CODE BEGIN #######
            min_range = self._closest_obstacle_in_front(msg)

            # cache latest front range for dynamic recovery verification
            self._latest_front_distance = min_range

            # check if the minimum range value is closer to the min_threshold_distance
            if min_range < self.min_threshold_distance:
                self._close_obstacle = True
                self.get_logger().info(f"Close obstacle detected at {min_range} meters")

            ####### TODO: ANSWER CODE END #######
        elif self._verifying_clearance:
            # Dynamic recovery: while verifying after a rotation, also update the
            # cached forward range so the control loop can decide whether to resume.
            ####### TODO: ANSWER CODE BEGIN #######
            self._latest_front_distance = self._closest_obstacle_in_front(msg)
            ####### TODO: ANSWER CODE END #######

    def start(self):
        """Wait for startup readiness and begin timer-driven control loop."""
        self._wait_for_sim_ready(STARTUP_TIMEOUT)
        self._control_timer = self.create_timer(1.0 / FREQUENCY, self._control_loop_callback)

    def _control_loop_callback(self):
        """Periodic control callback for random walk behavior."""
        # If the flag self._close_obstacle is False, the robot should move forward.
        # Otherwise, the robot should rotate for a random angle (use random.uniform() to generate a random value)
        # after which the flag is set again to False.
        # Use the function move to publish velocities already implemented,
        # passing the default velocities saved in the corresponding class members.

        ####### TODO: ANSWER CODE BEGIN #######

        # check if the robot is close to an obstacle

        if not self._close_obstacle:
            self.move(self.linear_velocity, 0.0)
            return

        # Dynamic recovery: after a rotation finishes, verify the FOV is clear
        # before resuming forward motion. If still blocked, pick a new rotation.
        if self._verifying_clearance:
            if self._latest_front_distance is None:
                return  # wait for a fresh scan
            if self._latest_front_distance >= self.min_threshold_distance:
                self._verifying_clearance = False
                self._close_obstacle = False
                self.get_logger().info(
                    f"Path clear at {self._latest_front_distance} meters. Resuming.")
            else:
                self.get_logger().info(
                    f"Still blocked at {self._latest_front_distance} meters. Re-rotating.")
                self._verifying_clearance = False
                # _close_obstacle stays True; next tick picks a new angle.
            return

        if self._rotation_target_time is None:

            random_angle_deg = random.uniform(self.random_turn_min_deg, self.random_turn_max_deg)
            random_angle_rad = math.radians(random_angle_deg)

            # determine the direction of the rotation

            direction = 1.0 if random_angle_rad >= 0.0 else -1.0
            self._rotation_angular_velocity = direction * self.angular_velocity

            # calculate the duration of the rotation
            rotation_duration = abs(random_angle_rad) / self.angular_velocity
            self._rotation_target_time = self.get_clock().now() + Duration(seconds=rotation_duration)

            self.get_logger().info(f"Starting rotation for {random_angle_deg} degrees")

        # either complete the rotation or continue rotating

        if self.get_clock().now() >= self._rotation_target_time:
            # Rotation done; enter verification instead of clearing the flag.
            self.stop()
            self._rotation_target_time = None
            self._verifying_clearance = True
            self._latest_front_distance = None
        else:
            self.move(0.0, self._rotation_angular_velocity)



        ####### TODO: ANSWER CODE END #######


def main(args=None):
    """Main function."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    random_walk = RandomWalk()
    try:
        random_walk.start()
        rclpy.spin(random_walk)
    except KeyboardInterrupt:
        if rclpy.ok():
            random_walk.get_logger().info('Ctrl+C received. Sending stop command...')
        random_walk.stop_and_flush()
    finally:
        if rclpy.ok():
            random_walk.stop_and_flush()
        random_walk.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    """Run the main function."""
    main()
