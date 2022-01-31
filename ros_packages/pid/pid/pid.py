import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3

class PID(Node):
    def __init__(self):
        super().__init__('pid')
        # Output of this node are the throttle and steering motor values [-1, 1]
        self.pid_pub = self.create_publisher(Vector3, 'motor_outputs', 10)

        # Inputs to this node are the GPS position / velocity estimates
        self.rtk_pos = self.create_subscription(Vector3, 'rtk_pos', self.rtk_pos_callback, 10)
        self.rtk_vel = self.create_subscription(Vector3, 'rtk_vel', self.rtk_vel_callback, 10)


    # Process new rtk position data
    def rtk_pos_callback(self, msg):
        pass

    # Process new rtk velocity data
    def rtk_vel_callback(self, msg):
        pass


    # Compute steering error (deviation from ideal path)
    def error(self):
        pass



# Run this ROS2 node
def main(args=None):
    rclpy.init(args=args)

    pid = PID()

    rclpy.spin(pid)

    pid.destroy_node()
    rclpy.shutdown()
