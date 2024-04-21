#!/usr/bin/env python3

# Import the simulation library
from drone_control.drone_controller import Controller

# Import the ROS 2 libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

# Import the ROS 2 message to received the current state of the drone
from nav_msgs.msg import Odometry

# Import my custom message to send the control to apply to the drone
from drone_msgs.msg import Control, WaypointReference

# Create a class that inherits the Drone object
class DroneControllerNode(Node):

    def __init__(self, ):
        super().__init__('drone_simulation_node', 
            allow_undeclared_parameters=True, 
            automatically_declare_parameters_from_overrides=True)

        # Get the mass, inertia and frequency from the configurations
        mass = self.get_parameter('drone_controller.mass').get_parameter_value().double_value
        frequency = self.get_parameter('drone_controller.frequency').get_parameter_value().double_value

        # Get the gains of the controller
        kp_pos = self.get_parameter('drone_controller.position.kp').get_parameter_value().double_array_value
        kd_pos = self.get_parameter('drone_controller.position.kd').get_parameter_value().double_array_value
        kp_theta = self.get_parameter('drone_controller.attitude.kp').get_parameter_value().double_value
        kd_theta = self.get_parameter('drone_controller.attitude.kd').get_parameter_value().double_value

        # Log the parameters
        self.get_logger().info("Drone mass: {}".format(mass))
        self.get_logger().info("Drone position gains: {}, {}".format(kp_pos, kd_pos))
        self.get_logger().info("Drone attitude gains: {}, {}".format(kp_theta, kd_theta))
        self.get_logger().info("Drone frequency: {}".format(frequency))

        # Create the object that actually simulates the dynamics of a drone
        self.controller = Controller(kp_pos, kd_pos, kp_theta, kd_theta, mass)

        # Get the current state of the system
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.x_dot = 0.0
        self.y_dot = 0.0
        self.theta_dot = 0.0

        # Get the desired waypoint
        self.x_target = 0.0
        self.y_target = 0.0

        # Publisher to the control inputs of the system
        self.control_publisher_ = self.create_publisher(Control, "input", qos_profile_sensor_data)

        # Subscriber to the current state of the system and for the reference waypoint for the controller to track
        self.state_subscriber_ = self.create_subscription(Odometry, "state", self.state_callback, qos_profile_system_default)
        self.waypoint_subscriber_ = self.create_subscription(WaypointReference, "waypoint", self.waypoint_callback, qos_profile_system_default)

        # Create a timer that will perform the update step of the simulation
        self.timer_ = self.create_timer(1.0/frequency, self.update_control)

    def state_callback(self, msg: Odometry):

        # Update the control input to apply to the system
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = msg.pose.pose.orientation.z

        self.x_dot = msg.twist.twist.linear.x
        self.y_dot = msg.twist.twist.linear.y
        self.theta_dot = msg.twist.twist.angular.z

    def waypoint_callback(self, msg: WaypointReference):

        # Update the waypoint to track
        self.x_target = msg.x
        self.y_target = msg.y

    def update_control(self):
        
        # Compute the control input to apply to the system
        T, omega = self.controller.update(self.x, self.y, self.theta, self.x_dot, self.y_dot, self.theta_dot, self.x_target, self.y_target)

        # Publish the control input
        control_msg = Control()
        control_msg.thrust = T
        control_msg.angular_velocity = omega
        self.control_publisher_.publish(control_msg)

def main(args=None):

    # Initialize the ROS 2 node
    rclpy.init(args=args)
    drone_controller = DroneControllerNode()
    rclpy.spin(drone_controller)

    # Destroy the node explicitly
    drone_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()