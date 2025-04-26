#!/usr/bin/env python3
"""
Advanced ROS Node for Line Following with Proportional Control

Features:
- Proportional control for line centering
- Configurable parameters via ROS parameters
- Comprehensive error handling
- Detailed logging
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float64
class LineController(Node):
    """Enhanced proportional controller for robust line following."""
    def __init__(self):
        """Initialize node with parameters, publishers, and subscriber."""
        super().__init__('line_controller')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_width', 640),
                ('gain_proportional', 0.1),
                ('base_linear_speed', 0.1),
                ('max_angular_speed', 1.0)
            ]
        )
        # Get parameter values
        self.image_width = self.get_parameter('image_width').value
        self.gain_proportional = self.get_parameter('gain_proportional').value
        self.base_linear_speed = self.get_parameter('base_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        # Validate parameters
        self._validate_parameters()
        # Initialize control variables
        self.target_x = self.image_width / 2
        self.last_error = 0.0
        # Setup publishers and subscriber
        self._setup_communications()
        self.get_logger().info(
            f"Line controller initialized with:\n"
            f"  Image width: {self.image_width}\n"
            f"  Proportional gain: {self.gain_proportional}\n"
            f"  Base speed: {self.base_linear_speed}\n"
            f"  Max angular: {self.max_angular_speed}"
        )

    def _validate_parameters(self):
        """Ensure parameters are within valid ranges."""
        if self.image_width <= 0:
            self.get_logger().warn("Invalid image_width, using default 640")
            self.image_width = 640
        if self.gain_proportional <= 0:
            self.get_logger().warn("Invalid gain_proportional, using default 0.1")
            self.gain_proportional = 0.1

    def _setup_communications(self):
        """Configure all ROS publishers and subscribers."""
        # Control command publisher
        self.twist_pub = self.create_publisher(
            Twist, 
            '/robot_twist', 
            10
        )
        # Error signal publisher
        self.error_pub = self.create_publisher(
            Float64, 
            '/control_error', 
            10
        )
        # Centroid position subscriber
        self.centroid_sub = self.create_subscription(
            PointStamped,
            '/image/centroid',
            self.centroid_callback,
            10
        )

    def centroid_callback(self, msg):
        """
        Process centroid position and compute control commands.
        
        Args:
            msg (PointStamped): Contains the detected line centroid position
        """
        try:
            # Calculate current error
            current_error = msg.point.x - self.target_x
            self.last_error = current_error
            
            # Publish error for monitoring
            self._publish_error(current_error)
            
            # Generate and publish control command
            control_msg = self._generate_control_command(current_error)
            self.twist_pub.publish(control_msg)
            
            self.get_logger().debug(
                f"Control Update - Error: {current_error:.2f}px, "
                f"Angular Z: {control_msg.angular.z:.2f}rad/s",
                throttle_duration_sec=0.5
            )
        except Exception as e:
            self.get_logger().error(f"Callback error: {str(e)}", throttle_duration_sec=1.0)

    def _publish_error(self, error):
        """Publish the current error to the error topic."""
        error_msg = Float64()
        error_msg.data = error
        self.error_pub.publish(error_msg)

    def _generate_control_command(self, error):
        """
        Generate Twist message based on current error.
        
        Args:
            error (float): Current error in pixels
            
        Returns:
            Twist: Control command message
        """
        cmd = Twist()
        
        # Set constant forward velocity
        cmd.linear.x = self.base_linear_speed
        
        # Calculate angular velocity (proportional control)
        angular_z = -self.gain_proportional * error
        
        # Apply angular speed limits
        cmd.angular.z = max(
            min(angular_z, self.max_angular_speed),
            -self.max_angular_speed
        )
        return cmd

def main(args=None):
    """Initialize and spin the node."""
    rclpy.init(args=args)
    try:
        controller = LineController()
        rclpy.spin(controller)
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
