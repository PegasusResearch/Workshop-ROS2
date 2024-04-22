#!/usr/bin/env python3

import os
import threading

# Import the ROS 2 dependencies
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from ament_index_python.packages import get_package_share_directory

# Import the odometry msgs to get the state of the vehicle and 
# my custom message to send the waypoint reference to the controller
from nav_msgs.msg import Odometry
from drone_msgs.msg import WaypointReference

# Import the GUI class
import tkinter as tk
from drone_gui.drone_window import GuiApp


class DroneGuiNode(Node):

    def __init__(self, gui_app):

        super().__init__('drone_gui_node',
            allow_undeclared_parameters=True, 
            automatically_declare_parameters_from_overrides=True)
    
        # Set the GUI app and the callback to set the waypoint
        self.gui_app = gui_app
        self.gui_app.set_waypoint = self.set_waypoint

        # Save the current states of the vehicles
        self.vehicles_state = {}

        # Subscribe to the state of the vehicle
        self.state_subscriber_ = self.create_subscription(Odometry, "state", self.state_callback, qos_profile_system_default)

        # Create a publisher to send the waypoint reference
        self.waypoint_publisher_ = self.create_publisher(WaypointReference, "waypoint", qos_profile_system_default)

    def set_waypoint(self, x, y):

        # Log the waypoint reference
        self.get_logger().info(f"Setting waypoint reference to ({x}, {y})")

        # Update the waypoint reference and publish it
        msg = WaypointReference()
        msg.reference = [float(x), float(y)]
        self.waypoint_publisher_.publish(msg)

    def state_callback(self, msg: Odometry):

        # Call the update method of the GUI to update the state of the vehicle
        self.gui_app.update_position_and_rotation(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z)


def main(args=None):

    # Initialize the ROS 2 context
    rclpy.init(args=args)

    # Get the assets directory
    assets_dir = os.path.join(get_package_share_directory('drone_gui'), 'assets')

    # Create the GUI application
    root = tk.Tk()
    root.resizable(False, False)
    app = GuiApp(root, assets_dir)

    # Create the node and start the main loop in a separate thread
    drone_gui_node = DroneGuiNode(app)
    thread_spin = threading.Thread(target=rclpy.spin, args=(drone_gui_node, ))
    thread_spin.start()

    # Start the GUI main loop
    root.mainloop()

    drone_gui_node.destroy_node()
    rclpy.shutdown()
    thread_spin.join()


if __name__ == '__main__':
    main()