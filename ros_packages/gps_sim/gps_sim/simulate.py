import rclpy
from rclpy.node import Node

from geographic_msgs.msg import GeoPoint


class GpsSimulator(Node):
    def __init__(self):
        super().__init__("GpsSimulator")
        self.publisher_ = self.create_publisher(GeoPoint, "rtk_pos", 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.f = open("/opt/ros/dev_ws/src/gps_sim/gps_sim/coordinates.txt", "r")

    def timer_callback(self):
        line = self.f.readline()
        if not line:
            # Repeat
            self.f.close()
            self.f = open("/opt/ros/dev_ws/src/gps_sim/gps_sim/coordinates.txt", "r")
            line = self.f.readline()
        strs = line.split(",")
        msg = GeoPoint()
        msg.altitude = 0.0
        msg.latitude = float(strs[0])
        msg.longitude = float(strs[1])

        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.altitude}, {msg.latitude}, {msg.longitude}")


def main(args=None):
    rclpy.init(args=args)

    gps_sim = GpsSimulator()

    rclpy.spin(gps_sim)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_sim.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
