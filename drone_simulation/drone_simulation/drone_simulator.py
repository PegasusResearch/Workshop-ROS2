#!/usr/bin/env python3
from math import cos, sin

class DroneSimulator:

    def __init__(self, mass, Ixx, dt):
        """
        mass: The mass of the drone
        Ixx: The momement of inertia 
        dt: The update step
        """
        self.m = mass
        self.Ixx = Ixx
        self.dt = dt

        self.g = 9.81

        # The initial state of the system
        self.x = 450.0
        self.y = 475.0
        self.theta = 0.0

        # The velocity of the system
        self.x_dot = 0.0
        self.y_dot = 0.0
        self.theta_dot = 0.0
        
    def update(self, u1, u2):
        """
        u1: The total thrust to apply 
        u2: The angular velocity
        """

        # Compute the second derivative of the state
        x_ddot = u1 / self.m * sin(self.theta)
        y_ddot = self.g - (u1 / self.m * cos(self.theta))
        theta_ddot = u2 / self.Ixx

        # Perform Euler Integration to get the current state
        self.x_dot += x_ddot * self.dt
        self.y_dot += y_ddot * self.dt
        self.theta_dot += theta_ddot * self.dt

        self.x += self.x_dot * self.dt
        self.y += self.y_dot * self.dt
        self.theta += self.theta_dot * self.dt

        # Make sure the drone does not go below the ground
        if self.y > 475.0:
            self.y = 475.0
            self.y_dot = 0.0

        return self.x, self.y, self.theta, self.x_dot, self.y_dot, self.theta_dot