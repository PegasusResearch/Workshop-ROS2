#!/usr/bin/env python3
import numpy as np

class Controller:

    def __init__(self, position_kp, position_kd, attitude_kp, attitude_kd, mass):

        self.g_force = np.array([0, -9.81])
        self.mass = mass

        # Initialize the horizontal gains
        self.x_kp = position_kp[0]
        self.x_kd = position_kd[0]

        # Initialize the vertical gains
        self.y_kp = position_kp[1]
        self.y_kd = position_kd[1]

        # Initialize the inner loop gains
        self.theta_kp = attitude_kp
        self.theta_kd = attitude_kd

    def update(self, x, y, theta, x_dot, y_dot, theta_dot, x_target, y_target):
        """
        x: The current x position of the drone
        y: The current y position of the drone
        theta: The current orientation of the drone
        x_dot: The current x velocity of the drone
        y_dot: The current y velocity of the drone
        theta_dot: The current angular velocity of the drone
        x_target: The x position to reach
        y_target: The y position to reach
        """
        
        # Compute the position error
        x_error = x_target - x
        y_error = y_target - y

        # Compute the outer loop control
        u_x = -self.x_kp * x_error - self.x_kd * x_dot
        u_y = -self.y_kp * y_error - self.y_kd * y_dot
        u = np.array([u_x, u_y])

        # Compute the total thrust
        T = self.mass * np.linalg.norm(u - self.g_force)

        # Compute the target angle
        normalized_u = -(u - self.g_force) * self.mass / T
        theta_target = np.arctan2(normalized_u[1], normalized_u[0])

        # Compute the inner loop control for the angular velocity
        omega = -self.theta_kp * (theta_target - theta) - self.theta_kd * theta_dot

        return T, omega
