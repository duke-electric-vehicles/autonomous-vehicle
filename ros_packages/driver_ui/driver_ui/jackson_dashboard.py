from geographic_msgs.msg import GeoPoint
import rclpy
from rclpy.node import Node
import time
from math import sin, cos, sqrt, atan2, radians

class DriverUI(Node):
    def __init__(self):
        super().__init__("driver_ui")

        # data sim
        self.create_subscription(
            GeoPoint, "gps_data_sim", self.position_callback_speed, 10
        )
        self.last_time = None
        self.last_lat = None
        self.last_lon = None
        self.rate_limit_seconds = 0.5  # Update every 0.5 seconds

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        # Radius of the Earth in km
        R = 6371.0

        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

        dlat = lat2 - lat1
        dlon = lon2 - lon1

        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))

        distance = R * c

        return distance  # in km

    def position_callback_speed(self, msg):
        current_time = time.time()

        if self.last_time is not None and (current_time - self.last_time) >= self.rate_limit_seconds:
            time_diff = current_time - self.last_time  # in seconds
            distance = self.haversine_distance(self.last_lat, self.last_lon, msg.latitude, msg.longitude)  # in km
            speed = (distance * 1000) / time_diff  # convert km to m, speed in m/s

            print(f"Lat: {msg.latitude}, Lon: {msg.longitude}, Speed: {speed} m/s")
            self.last_time = current_time

        if self.last_time is None:
            self.last_time = current_time

        self.last_lat = msg.latitude
        self.last_lon = msg.longitude

def main(args=None):
    rclpy.init(args=args)

    driver_ui = DriverUI()

    rclpy.spin(driver_ui)

    driver_ui.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
