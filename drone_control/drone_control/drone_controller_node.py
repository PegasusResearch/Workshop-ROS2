#!/usr/bin/env python3

# Import the simulation library
from drone_control.drone_controller import Controller

# Import the ROS 2 libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

# Import the ROS 2 message to received the current state of the drone
from geometry_msgs.msg import PoseStamped

# Import my custom message to send the control to apply to the drone
from drone_msgs.msg import Control

# Create a class that inherits the Drone object
class DroneControllerNode(Node):

    def __init__(self, ):
        super().__init__('drone_simulation_node', 
            allow_undeclared_parameters=True, 
            automatically_declare_parameters_from_overrides=True)

        # Get the mass, inertia and frequency from the configurations
        mass = self.get_parameter('drone_simulation.mass').get_parameter_value().double_value
        Ixx = self.get_parameter('drone_simulation.Ixx').get_parameter_value().double_value
        frequency = self.get_parameter('drone_simulation.frequency').get_parameter_value().double_value

        # Log the parameters
        self.get_logger().info("Drone mass: {}".format(mass))
        self.get_logger().info("Drone Ixx: {}".format(Ixx))
        self.get_logger().info("Drone frequency: {}".format(frequency))

        # Create the object that actually simulates the dynamics of a drone
        self.controller = Controller()

        # Get the current state of the system
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Publisher to the control inputs of the system
        self.control_subscriber_ = self.create_publisher(Control, "input", self.state_callback, qos_profile_sensor_data)

        # Subscriber to the current state of the system
        self.state_publisher_ = self.create_subscription(PoseStamped, "state", qos_profile_system_default)

        # Create a timer that will perform the update step of the simulation
        self.timer_ = self.create_timer(1.0/frequency, self.update_control)

    def state_callback(self, msg: PoseStamped):

        # Update the control input to apply to the system
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.theta = msg.pose.orientation.z

    def update_control(self):
        pass

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