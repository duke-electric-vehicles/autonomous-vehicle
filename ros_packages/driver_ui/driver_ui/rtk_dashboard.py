from geometry_msgs.msg import Vector3
from geographic_msgs.msg import GeoPoint
import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk

class DriverUI(Node):
    def __init__(self):
        super().__init__("driver_ui")

        self.position = None
        self.velocity = None

        # Create the tkinter window
        self.root = tk.Tk()
        self.root.title("Driver UI")

        # Position labels
        ttk.Label(self.root, text="Latitude:").grid(row=0, column=0)
        self.lat_label = ttk.Label(self.root, text="")
        self.lat_label.grid(row=0, column=1)

        ttk.Label(self.root, text="Longitude:").grid(row=1, column=0)
        self.lon_label = ttk.Label(self.root, text="")
        self.lon_label.grid(row=1, column=1)

        ttk.Label(self.root, text="Altitude:").grid(row=2, column=0)
        self.alt_label = ttk.Label(self.root, text="")
        self.alt_label.grid(row=2, column=1)

        # Velocity labels
        ttk.Label(self.root, text="Velocity x:").grid(row=3, column=0)
        self.vel_x_label = ttk.Label(self.root, text="")
        self.vel_x_label.grid(row=3, column=1)

        ttk.Label(self.root, text="Velocity y:").grid(row=4, column=0)
        self.vel_y_label = ttk.Label(self.root, text="")
        self.vel_y_label.grid(row=4, column=1)

        ttk.Label(self.root, text="Velocity z:").grid(row=5, column=0)
        self.vel_z_label = ttk.Label(self.root, text="")
        self.vel_z_label.grid(row=5, column=1)

        self.create_subscription(GeoPoint, "rtk_pos", self.position_callback, 10)
        self.create_subscription(Vector3, "rtk_vel", self.vel_callback, 10)

        # Periodically update the GUI
        self.root.after(100, self.update_gui)

    def position_callback(self, msg):
        self.get_logger().info('Received position: Latitude: %f, Longitude: %f, Altitude: %f' % (msg.latitude, msg.longitude, msg.altitude))
        self.position = msg

    def vel_callback(self, msg):
        self.get_logger().info('Received velocity: x: %f, y: %f, z: %f' % (msg.x, msg.y, msg.z))
        self.velocity = msg

    def update_gui(self):
        # Update position labels
        if self.position:
            self.lat_label["text"] = str(self.position.latitude)
            self.lon_label["text"] = str(self.position.longitude)
            self.alt_label["text"] = str(self.position.altitude)
        
        if self.velocity:
            self.vel_x_label["text"] = str(self.velocity.x)
            self.vel_y_label["text"] = str(self.velocity.y)
            self.vel_z_label["text"] = str(self.velocity.z)
        self.root.after(100, self.update_gui)

    def run_tkinter(self):
        self.root.update()

def main(args=None):
    rclpy.init(args=args)
    driver_ui = DriverUI()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(driver_ui, timeout_sec=0.1)  # Adjust the timeout as needed
            driver_ui.run_tkinter()
    except KeyboardInterrupt:
        pass
    finally:
        driver_ui.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()