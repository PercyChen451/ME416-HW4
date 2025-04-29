#!/usr/bin/env python3
"""
ROS Node for Line Following Controller with PID Control

This node implements a PID controller to keep a detected line at the center
of the camera image by adjusting the robot's angular velocity.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float64
from rclpy.time import Time

class PID:
    """Simple PID controller implementation."""
    def __init__(self, kp=0.0, kd=0.0, ki=0.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.prev_error = 0.0
        self.integral = 0.0

    def proportional(self, error):
        return self.kp * error

    def derivative(self, error, dt):
        if dt > 0:
            derivative = (error - self.prev_error) / dt
            self.prev_error = error
            return self.kd * derivative
        return 0.0

    def integral(self, error, dt):
        self.integral += error * dt
        return self.ki * self.integral

class LineController(Node):
    """PID controller for line following."""
    
    def __init__(self):
        """Initialize the node, publishers, subscriber, and control parameters."""
        super().__init__('line_controller')
        
        # 1) Initialize attributes (all set to zero for now)
        self.lin_speed = 0.0
        self.gain_proportional = 0.0
        self.gain_derivative = 0.0
        self.gain_integral = 0.0
        
        # Camera image width (pixels)
        self.image_width = 640
        
        # 3) Initialize PID controller
        self.pid = PID(
            kp=self.gain_proportional,
            kd=self.gain_derivative,
            ki=self.gain_integral)
        
        # 4) Initialize previous message attribute
        self.msg_previous = None
        
        # 2) Initialize publishers
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_pub = self.create_publisher(Float64, '/control_error', 10)
        
        # 2) Initialize subscriber
        self.centroid_sub = self.create_subscription(
            PointStamped,
            '/image/centroid',
            self.centroid_callback,
            10)
        
        self.get_logger().info('Line controller node initialized')

    def centroid_callback(self, msg):
        """
        Callback for processing centroid position and computing control output.
        
        Args:
            msg (PointStamped): Message containing the x-coordinate of the detected line centroid
        """
        try:
            # 1) Compute error signal
            error_signal = msg.point.x - (self.image_width / 2)
            
            # 2) Publish error signal
            error_msg = Float64()
            error_msg.data = error_signal
            self.error_pub.publish(error_msg)
            
            # 3) Calculate time delay
            time_delay = 0.0
            if self.msg_previous is not None:
                time_delay = self.stamp_difference(msg.header.stamp, self.msg_previous.header.stamp)
            
            # 4) Initialize Twist message
            msg_twist = Twist()
            
            # 5) Set linear speed
            msg_twist.linear.x = self.lin_speed
            
            # 6) Set angular speed using PID controller
            angular_z = (self.pid.proportional(error_signal) +
                         self.pid.derivative(error_signal, time_delay) +
                         self.pid.integral(error_signal, time_delay))
            msg_twist.angular.z = float(angular_z)
            
            # 7) Publish Twist message
            self.twist_pub.publish(msg_twist)
            
            # Update previous message
            self.msg_previous = msg
            
            self.get_logger().debug(
                f"Error: {error_signal:.2f}, Angular Z: {msg_twist.angular.z:.2f}",
                throttle_duration_sec=1)
            
        except Exception as e:
            self.get_logger().error(f"Error in centroid callback: {str(e)}")

    @staticmethod
    def stamp_difference(stamp2, stamp1):
        """Calculate time difference between two stamps in seconds"""
        time1 = Time.from_msg(stamp1)
        time2 = Time.from_msg(stamp2)
        duration = time2 - time1
        return duration.nanoseconds / 1e9

def main(args=None):
    """Main entry point for the ROS node."""
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
