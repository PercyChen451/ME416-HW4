#!/usr/bin/env python
""" A first template for a PID controller """
class PID:
    """PID controller implementation."""
    def __init__(self, kp=0.0, kd=0.0, ki=0.0):
        """Initialize PID controller with gains."""
        self.kp = float(kp)
        self.kd = float(kd)
        self.ki = float(ki)
        self.prev_error = 0.0
        self.integral = 0.0
    def update(self, error, dt): """Calculate all PID terms at once."""
        p_term = self.kp * error
        d_term = 0.0
        if dt > 0:
            d_term = self.kd * (error - self.prev_error) / dt
            self.prev_error = error
        self.integral += error * dt
        i_term = self.ki * self.integral
        return p_term + d_term + i_term
