#!/usr/bin/env python3

# Import the simulation library
from drone_simulation.drone_simulator import DroneSimulator

# Import the ROS 2 libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

# Import the ROS 2 message to publish the current state of the drone
from geometry_msgs.msg import PoseStamped

# Import my custom message to receive the control to apply to the drone
from drone_msgs.msg import Control

# Create a class that inherits the Drone object
class DroneSimulationNode(Node):

    def __init__(self, ):
        super().__init__('drone_simulation_node', 
            allow_undeclared_parameters=True, 
            automatically_declare_parameters_from_overrides=True)

        # Get the mass, inertia and frequency from the configurations
        mass = self.get_parameter('drone_simulation.mass').get_parameter_value().integer_value
        Ixx = self.get_parameter('drone_simulation.Ixx').get_parameter_value().double_value
        frequency = self.get_parameter('drone_simulation.frequency').get_parameter_value().double_value

        # Create the object that actually simulates the dynamics of a drone
        self.simulator = DroneSimulator(mass, Ixx, 1.0/frequency)

        # Setup the input of the drone simulator
        self.u1 = 0.0
        self.u2 = 0.0

        # Subscribe to the control inputs of the system
        self.control_subscriber_ = self.create_subscription(Control, "input", self.control_callback, qos_profile_sensor_data)

        # Publisher to output the simulator data
        self.state_publisher_ = self.create_publisher(PoseStamped, "state", qos_profile_system_default)

        # Create a timer that will perform the update step of the simulation
        self.timer_ = self.create_timer(1.0/frequency, self.update_simulation)

    def control_callback(self, msg: Control):

        # Update the control input to apply to the system
        self.u1 = msg.u1
        self.u2 = msg.u2

    def update_simulation(self):
        
        # Update the current state of the system
        x, y, theta = self.simulator.update(self.u1, self.u2)

        # Create a PoseStamped message
        msg = PoseStamped()
        msg.header.frame_id = "world"  # or map, or inertial frame (depending on your background)
        msg.header.stamp = self.get_clock().now().to_msg()

        # Set the position of the drone
        msg.pose.position.x = x
        msg.pose.position.y = y

        # Set the orientation of the drone
        msg.pose.orientation.z = theta

        # Publish the message
        self.state_publisher_.publish(msg)

def main(args=None):

    # Initialize the ROS 2 node
    rclpy.init(args=args)
    drone_simulator = DroneSimulationNode()
    rclpy.spin(drone_simulator)

    # Destroy the node explicitly
    drone_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()