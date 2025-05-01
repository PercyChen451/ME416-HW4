#!/usr/bin/env python3
"""
ROS Node for Line Following Controller with PID Control
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float64
from rclpy.time import Time
class PID:
    """PID controller implementation."""
    def __init__(self, kp=0.0, kd=0.0, ki=0.0):
        """Initialize PID controller with gains."""
        self.kp = float(kp)
        self.kd = float(kd)
        self.ki = float(ki)
        self.prev_error = 0.0
        self.integral = 0.0
    def update(self, error, dt):
        """Calculate all PID terms at once."""
        p_term = self.kp * error
        d_term = 0.0
        if dt > 0:
            d_term = self.kd * (error - self.prev_error) / dt
            self.prev_error = error
        self.integral += error * dt
        i_term = self.ki * self.integral
        return p_term + d_term + i_term
class LineController(Node):
    def __init__(self):
        """Initialize node with parameters and publishers."""
        super().__init__('line_controller')
        self.lin_speed = 0.1  # Set to small non-zero value for testing
        self.gain_proportional = 0.5  # Start with P term only
        self.gain_derivative = 0.0
        self.gain_integral = 0.0
        self.image_width = 640
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
        """Process centroid messages and compute control output."""
        try:
            error_signal = float(msg.point.x - (self.image_width / 2))
            error_msg = Float64()
            error_msg.data = error_signal
            self.error_pub.publish(error_msg)
            time_delay = 0.1  # Start with fixed small value for testing
            if self.msg_previous is not None:
                time_delay = self.stamp_difference(msg.header.stamp, self.msg_previous.header.stamp)
            msg_twist = Twist()
            msg_twist.linear.x = float(self.lin_speed)
            pid_output = self.pid.update(error_signal, time_delay)
            msg_twist.angular.z = float(pid_output)
            self.cmd_vel_pub.publish(msg_twist)
            self.msg_previous = msg
            self.get_logger().info(
                f"Error: {error_signal:.1f}, Output: {pid_output:.2f}",
                throttle_duration_sec=0.5)
        except Exception as e:
            self.get_logger().error(f"Error in callback: {str(e)}")
    @staticmethod
    def stamp_difference(stamp2, stamp1):
        """Calculate time difference between two stamps in seconds."""
        time1 = Time.from_msg(stamp1)
        time2 = Time.from_msg(stamp2)
        duration = time2 - time1
        return duration.nanoseconds / 1e9
def main(args=None):
    """Main function to initialize and run the node."""
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
