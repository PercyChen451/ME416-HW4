'''
Modeling and utilities for differential drive robots

'''
import math as m

import numpy as np
from me416_utilities import stamp_difference


def model_parameters():
    '''
    Return the lumped parameters for the differential drive model
    '''
    gain_k = 1.0
    gain_d = 0.7
    return gain_k, gain_d


def closed_form_parameters(z_zero, u_input):
    '''
    Return parameters of the closed form solution for constant-speeds arcs
    '''
    gain_k, gain_d = model_parameters()

    speed_lw, speed_rw = u_input[0, 0], u_input[1, 0]
    x_0 = z_zero[0, 0]
    y_0 = z_zero[1, 0]
    theta0 = z_zero[2, 0]

    radius = gain_d * (speed_rw + speed_lw) / (speed_rw - speed_lw)
    omega = gain_k * (speed_rw - speed_lw) / (2.0 * gain_d)
    c_x = x_0 - radius * m.sin(theta0)
    c_y = y_0 + radius * m.cos(theta0)
    c_theta = theta0

    return radius, omega, c_x, c_y, c_theta


def closed_form_step(z_current, u_input, t_next):
    """
    z: 3x1, current state z0, [[X0], [Y0], [theta0]]
    u: 2x1, [[speed_lw], [speed_rw]]
    t: float, time at which to compute the solution
    return zp: 3x1, z(T)
    """
    gain_k, _ = model_parameters()
    if t_next is None:
        t_next = 0

    speed_lw, speed_rw = u_input[0, 0], u_input[1, 0]
    x_0 = z_current[0, 0]
    y_0 = z_current[1, 0]
    theta0 = z_current[2, 0]
    if abs(speed_lw - speed_rw) < 1e-5:
        z_next = np.array([[
            gain_k * (speed_rw + speed_lw) / 2.0 * m.cos(theta0) * t_next + x_0
        ], [
            gain_k * (speed_rw + speed_lw) / 2.0 * m.sin(theta0) * t_next + y_0
        ], [theta0]])
    else:
        radius, omega, c_x, c_y, c_theta = closed_form_parameters(
            z_current, u_input)
        z_next = np.array([[radius * m.sin(omega * t_next + c_theta) + c_x],
                           [-radius * m.cos(omega * t_next + c_theta) + c_y],
                           [omega * t_next + c_theta]])

    return z_next


def clamp(val):
    """Clamp a value between -1 and 1"""
    return max(-1.0, min(1.0, val))


def twist_to_speeds(speed_linear, speed_angular):
    """
    Given the desired linear and angular speeds for the robot,
    returns the left and right motor speeds
    """
    gain_k, gain_d = model_parameters()
    left = (speed_linear - (speed_angular * gain_d)) / gain_k
    right = (speed_linear + (speed_angular * gain_d)) / gain_k
    return clamp(left), clamp(right)


def system_matrix(theta):
    """
    Return the matrix A from the dynamics of the differential drive model
    """
    gain_k, gain_d = model_parameters()
    a_matrix = (gain_k / 2.0) * np.array([[
        m.cos(theta), m.cos(theta)
    ], [m.sin(theta), m.sin(theta)], [-1 / gain_d, 1 / gain_d]])
    return a_matrix


def system_field(z_current, u_input):
    '''
    Return the derivative of the state according to the differential drive model
    '''
    theta = z_current[2][0]
    a_matrix = system_matrix(theta)
    dot_z = a_matrix @ u_input
    return dot_z


def euler_step(z_current, u_current, step_size):
    '''
    Update the state estimate using one step of the Euler method
    '''
    dot_z = system_field(z_current, u_current)
    z_next = z_current + step_size * dot_z
    return z_next


class KeysToVelocities():
    '''
    Class to translate cumulative key strokes to speed commands
    '''

    def __init__(self):
        self.speed_linear = 0.0
        self.speed_angular = 0.0
        self.speed_delta = 0.2
        self.text_description = ''

    def change_linear(self, amount):
        """Wrapper to change linear velocity"""
        if amount is None:
            self.speed_linear = 0.
        else:
            self.speed_linear += amount
            self.speed_linear = clamp(self.speed_linear)

    def change_angular(self, amount):
        """Wrapper to change angular velocity"""
        if amount is None:
            self.speed_angular = 0.
        else:
            self.speed_angular += amount
            self.speed_angular = clamp(self.speed_angular)

    def zero_linear_angular(self, _):
        """Wrapper to change angular velocity"""
        self.speed_linear = 0.
        self.speed_angular = 0.

    def update_speeds(self, key):
        '''
        Given a key, call the corresponding wrapper and return the
        corresponding string
        '''
        # normalize key to lowercase (makes the function case invariant)
        key = key.lower()

        # dictionary mapping a key to a function, its arguments, and a message
        # This approach is more indirect, it can be easily expanded to new keys
        actions = {
            'w': (self.change_linear, +self.speed_delta,
                  f'Linear speed +{self.speed_delta}'),
            's': (self.change_linear, -self.speed_delta,
                  f'Linear speed -{self.speed_delta}'),
            'a': (self.change_angular, +self.speed_delta,
                  f'Angular speed +{self.speed_delta}'),
            'd': (self.change_angular, -self.speed_delta,
                  f'Angular speed -{self.speed_delta}'),
            'z': (self.change_linear, None, 'Linear speed =0.0'),
            'c': (self.change_angular, None, 'Angular speed =0.0'),
            'x': (self.zero_linear_angular, None, 'Zeroed all speeds')
        }

        # Check if it is a valid key
        if key in actions:
            # if the key is in the action dictionary, call the function
            # with the arguments and store the message
            action_set = actions[key]
            action_set[0](action_set[1])
            self.text_description = action_set[2]

            # add current speeds to message
            self.text_description = self.text_description + ". " + \
                f"Current speeds: linear={self.speed_linear}, " + \
                f"angular={self.speed_angular}."
        else:
            self.text_description = 'Invalid Key Press'
        return self.speed_linear, self.speed_angular, self.text_description


class StampedMsgRegister():
    '''
    Store a previous message, and compute delay with respect to current one
    '''

    def __init__(self):
        self.msg_previous = None

    def replace_and_compute_delay(self, msg):
        '''
        Compute delay between current and stored message,
        then replace stored message with current one
        '''
        if self.msg_previous is None:
            msg_previous = None
            self.msg_previous = msg
            time_delay = None
        else:
            msg_previous = self.msg_previous
            self.msg_previous = msg
            time_delay = stamp_difference(msg.header.stamp,
                                          msg_previous.header.stamp)

        return time_delay, msg_previous

    def previous_stamp(self):
        '''
        Return stored message
        '''
        if self.msg_previous is None:
            stamp = None
        else:
            stamp = self.msg_previous.header.stamp
        return stamp
