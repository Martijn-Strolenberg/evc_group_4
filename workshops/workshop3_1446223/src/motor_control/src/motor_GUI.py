# -*- coding: utf-8 -*-
import Tkinter as tk  # For Python 2


class MotorGUI:
    def __init__(self):
    # USER DEFINED DEFAULTS
        self.def_velocity = 1    # Velocity in rad / sec
        self.def_distance = 1    # Distance in m
        self.def_angle_deg = 1   # Angle in degrees

        # Create main window
        root = tk.Tk()
        root.title("Input GUI")

        # Velocity input
        tk.Label(root, text="Velocity (rad/s):").grid(row=0, column=0, padx=10, pady=5, sticky="e")
        self.velocity_entry = tk.Entry(root)
        self.velocity_entry.grid(row=0, column=1, pady=5)
        self.velocity_entry.insert(0, str(self.def_velocity))

        # Distance input
        tk.Label(root, text="Distance (m):").grid(row=1, column=0, padx=10, pady=5, sticky="e")
        self.distance_entry = tk.Entry(root)
        self.distance_entry.grid(row=1, column=1, pady=5)
        self.distance_entry.insert(0, str(self.def_distance))

        # Angle input
        tk.Label(root, text="Angle (deg):").grid(row=2, column=0, padx=10, pady=5, sticky="e")
        self.angle_entry = tk.Entry(root)
        self.angle_entry.grid(row=2, column=1, pady=5)
        self.angle_entry.insert(0, str(self.def_angle_deg))

        # Submit button
        submit_btn = tk.Button(root, text="Submit", command=self.submit)
        submit_btn.grid(row=3, columnspan=2, pady=10)

        # Run the GUI loop
        root.mainloop()

    def submit(self):
        self.velocity = self.velocity_entry.get()
        self.distance = self.distance_entry.get()
        self.angle = self.angle_entry.get()

        print("Velocity (rad/s):", self.velocity)
        print("Distance (m):", self.distance)
        print("Angle (deg):", self.angle)


