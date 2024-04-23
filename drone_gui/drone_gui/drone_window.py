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

        # Set an empty callback to set the waypoint reference
        self.set_waypoint = None
        
        # Initial position and rotation angle
        self.vehicle_states = {}

        # Bind mouse click event to canvas
        self.canvas.bind("<Button-1>", self.place_image_at_click)

        self.last_time = 0
        

    def update_position_and_rotation(self, x, y, angle, vehicle_id, curr_time):

        if curr_time - self.last_time < 0.015:
            return
        
        self.last_time = curr_time

        # Delete the previous image and text
        if vehicle_id not in self.vehicle_states:
            self.vehicle_states[vehicle_id] = {}
        
        # Rotate the image and place it on the canvas
        rotated_image = self.image.rotate(-angle * 180.0 / 3.14159, expand=True)
        self.vehicle_states[vehicle_id]["image"] = ImageTk.PhotoImage(rotated_image)
        new_image = self.canvas.create_image(x, y, image=self.vehicle_states[vehicle_id]["image"], anchor=tk.CENTER)
        new_text = self.canvas.create_text(x, y - 40, text=vehicle_id, font=("Arial", 12), fill="black")

        if vehicle_id in self.vehicle_states and "text" in self.vehicle_states[vehicle_id] and "image" in self.vehicle_states[vehicle_id]:
            
            # Delete the previous text and image
            self.canvas.delete(self.vehicle_states[vehicle_id]["text"])
            self.canvas.delete(self.vehicle_states[vehicle_id]["image_item"])

        self.vehicle_states[vehicle_id]["text"] = new_text
        self.vehicle_states[vehicle_id]["image_item"] = new_image

    def place_image_at_click(self, event):

        if self.set_waypoint is not None:

            # Call the callback to set the waypoint
            self.set_waypoint(event.x, event.y)
