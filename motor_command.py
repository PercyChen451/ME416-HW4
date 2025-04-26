#!/usr/bin/env python
'''
Listen to Twist messages, set motor speeds
'''
import me416_utilities as mu
import rclpy
import robot_model
from geometry_msgs.msg import Twist
from me416_msgs.msg import MotorSpeedsStamped
from rclpy.node import Node

# The motors will most likely not spin exactly at the same speed,
# this is a simple factor to attempt to account for this.
# multiply the faster motor by this offset.
SPEED_MULTIPLIER = 1.0


class MotorCommand(Node):
    '''
    A node to control the motor from messages
    '''

    def __init__(self):
        '''
        Configure Twist subscriber, MotorSpeeds publisher,
        and objects to control the motors
        '''
        super().__init__('motor_command')
        self.publish_speeds = self.create_publisher(MotorSpeedsStamped,
                                                    'motor_speeds', 10)

        self.subscriber = self.create_subscription(Twist, 'cmd_vel',
                                                   self.translate_twist, 10)

        # initialize motor speeds
        self.left_motor = mu.MotorSpeedLeft(SPEED_MULTIPLIER)
        self.right_motor = mu.MotorSpeedRight()

    def translate_twist(self, twist_msg):
        '''
        Gets a Twist message and computes and applies
        the corresponding speeds to the two motors.
        '''
        # extract rigid body velocities from message
        linear = twist_msg.linear.x
        angular = twist_msg.angular.z
        # mix velocities to compute motor speeds
        left, right = robot_model.twist_to_speeds(linear, angular)
        # send motor speeds to the motors
        self.left_motor.set_speed(left)
        self.right_motor.set_speed(right)
        # prepare and publish motor speeds
        msg = MotorSpeedsStamped()
        msg.left = left
        msg.right = right
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publish_speeds.publish(msg)


def main(args=None):
    '''
    Initialize and spin node
    '''

    rclpy.init(args=args)
    motor_manager = MotorCommand()
    rclpy.spin(motor_manager)
    motor_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
