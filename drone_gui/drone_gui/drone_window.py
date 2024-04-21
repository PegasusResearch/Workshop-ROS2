import os
import tkinter as tk
from PIL import Image, ImageTk

class GuiApp:

    def __init__(self, master, assets_dir="assets"):
        self.master = master
        self.master.title("Drone GUI")

        self.assets_dir = assets_dir
        
        # Light blue background
        self.canvas = tk.Canvas(self.master, bg="light blue", width=900, height=500)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Draw brown rectangle (the ground)
        self.canvas.create_rectangle(0, 490, 901, 501, fill="brown")
        
        # Load image
        self.image = Image.open(os.path.join(self.assets_dir, "drone1.png"))
        self.image = self.image.resize((50, 50), Image.ANTIALIAS)
        self.photo = ImageTk.PhotoImage(self.image)

        # Set an empty callback to set the waypoint reference
        self.set_waypoint = None
        
        # Initial position and rotation angle
        self.x = 0
        self.y = 460
        self.angle = 0
        
        # Place image on canvas
        self.image_item = self.canvas.create_image(self.x, self.y, image=self.photo, anchor=tk.CENTER)

        # Bind mouse click event to canvas
        self.canvas.bind("<Button-1>", self.place_image_at_click)
        
        # Start updating position and rotation
        self.update_position_and_rotation(450,475,0)
        

    def update_position_and_rotation(self, x, y, angle):

        # Update position (simple example, you can modify this logic)
        self.x = x
        self.y = y
        self.angle = angle * 180 / 3.14159  # Convert from radians to degrees
        
        # Update image position and rotation
        self.canvas.delete(self.image_item)  # Remove previous image
        rotated_image = self.image.rotate(self.angle, expand=True)
        self.photo = ImageTk.PhotoImage(rotated_image)
        self.image_item = self.canvas.create_image(self.x, self.y, image=self.photo, anchor=tk.CENTER)

    def place_image_at_click(self, event):

        if self.set_waypoint is not None:

            # Call the callback to set the waypoint
            self.set_waypoint(event.x, event.y)
