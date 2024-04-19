#!/usr/bin/env python3

class Controller:

    def __init__(self):
        pass

    def update(self, x, y, theta, x_target, y_target, theta_target):
        """
        x: The current x position of the drone
        y: The current y position of the drone
        theta: The current orientation of the drone
        x_target: The x position to reach
        y_target: The y position to reach
        theta_target: The orientation to reach
        """
        
        # Compute the error in the position
        x_error = x_target - x
        y_error = y_target - y

        