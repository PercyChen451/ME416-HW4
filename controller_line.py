#!/usr/bin/env python3
"""
ROS Node for Line Following Controller with PID Control

This node implements a PID controller to maintain a robot on a visual line by:
1. Subscribing to centroid coordinates from image processing
2. Calculating error from desired position
3. Computing PID control output
4. Publishing motor commands
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float64
class PID:
    """PID controller implementation for angular velocity correction."""
    def __init__(self, kp=0.0, kd=0.0, ki=0.0):
        """
        Initialize PID controller with gains.
        Args:
            kp (float): Proportional gain
            kd (float): Derivative gain
            ki (float): Integral gain
        """
        self.kp = float(kp)
        self.kd = float(kd)
        self.ki = float(ki)
        self.prev_error = 0.0
        self.integral = 0.0
    def update(self, error, dt):
        """
        Calculate PID control output.
        Args:
            error (float): Current error from setpoint
            dt (float): Time delta since last update
        Returns:
            float: PID control output
        """
        p_term = self.kp * error
        d_term = 0.0
        if dt > 0:
            d_term = self.kd * (error - self.prev_error) / dt
            self.prev_error = error
        self.integral += error * dt
        i_term = self.ki * self.integral
        return p_term + d_term + i_term
class LineController(Node):
    """ROS node for visual line following using PID control."""
    def __init__(self):
        """
        Initialize the line controller node with:
        - PID parameters
        - Image processing settings
        - ROS publishers/subscribers
        """
        super().__init__('line_controller')
        self.lin_speed = 0.1  # Constant forward speed (m/s)
        self.image_width = 640  # Camera image width in pixels
        self.gain_proportional = 0.5
        self.gain_derivative = 0.0
        self.gain_integral = 0.0
        self.pid = PID(
            kp=self.gain_proportional,
            kd=self.gain_derivative,
            ki=self.gain_integral)
        self.msg_previous = None
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_pub = self.create_publisher(Float64, '/control_error', 10)
        self.centroid_sub = self.create_subscription(
            PointStamped,
            '/image/centroid',
            self.centroid_callback,
            10)
        self.get_logger().info('Line controller node initialized')
    def centroid_callback(self, msg):
        """
        Process centroid position and compute control output.
        Args:
            msg (PointStamped): Message containing line centroid coordinates
        """
        try:
            error_signal = float(msg.point.x - (self.image_width / 2))
            error_msg = Float64()
            error_msg.data = error_signal
            self.error_pub.publish(error_msg)
            time_delay = 0.1  # Default small value
            if self.msg_previous is not None:
                time_delay = self.stamp_difference(
                    msg.header.stamp,
                    self.msg_previous.header.stamp)
            msg_twist = Twist()
            msg_twist.linear.x = float(self.lin_speed)
            pid_output = self.pid.update(error_signal, time_delay)
            msg_twist.angular.z = float(pid_output)
            self.cmd_vel_pub.publish(msg_twist)
            self.msg_previous = msg
            self.get_logger().info(
                f"Error: {error_signal:.1f}, Output: {pid_output:.2f}",
                throttle_duration_sec=0.5)
        except (ValueError, AttributeError) as e:
            self.get_logger().error(
                f"Control error: {str(e)}",
                throttle_duration_sec=1.0)
    @staticmethod
    def stamp_difference(stamp2, stamp1):
        """
        Calculate time difference between two ROS timestamps.
        Args:
            stamp1: First timestamp
            stamp2: Second timestamp
        Returns:
            float: Time difference in seconds
        """
        time1 = Time.from_msg(stamp1)
        time2 = Time.from_msg(stamp2)
        duration = time2 - time1
        return duration.nanoseconds / 1e9
def main(args=None):
    """Initialize and run the line controller node."""
    rclpy.init(args=args)
    controller = LineController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
