import rclpy
from rclpy.node import Node

from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Int64


class RtkDistance(Node):
    def __init__(self):
        super().__init__("rtk_distance")
        self.dist_publisher = self.create_publisher(Int64, "rtk_dist", 10)
        self.subscription = self.create_subscription(
            GeoPoint, "rtk_pos", self.pos_listener, 10
        )

    def pos_listener(self, msg, **metadata):
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude


def main(args=None):
    rclpy.init(args=args)
    rtk_distance = RtkDistance()

    rclpy.spin(rtk_distance)

    rtk_distance.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
