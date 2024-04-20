#!/usr/bin/env python3

import os
import threading

# Import the ROS 2 dependencies
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# Import the GUI class
import tkinter as tk
from drone_gui.drone_window import GuiApp


class DroneGuiNode(Node):

    def __init__(self):
        super().__init__('drone_gui_node')


def main(args=None):

    # Initialize the ROS 2 context
    rclpy.init(args=args)

    # Create the node and start the main loop in a separate thread
    drone_gui_node = DroneGuiNode()
    thread_spin = threading.Thread(target=rclpy.spin, args=(drone_gui_node, ))
    thread_spin.start()

    # Get the assets directory
    assets_dir = os.path.join(get_package_share_directory('drone_gui'), 'assets')

    # Create the GUI application
    root = tk.Tk()
    root.resizable(False, False)
    app = GuiApp(root, assets_dir)
    root.mainloop()

    drone_gui_node.destroy_node()
    rclpy.shutdown()
    thread_spin.join()


if __name__ == '__main__':
    main()