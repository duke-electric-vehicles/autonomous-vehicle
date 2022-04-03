import rclpy
from rclpy.node import Node
import csv

#from std_msgs.msg import String
from geometry_msgs.msg import Vector3


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Vector3, 'rtk_pos', 10)
        timer_period = 1/7.0  # seconds modeled by gps 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.path_file = "/opt/ros/dev_ws/src/driver_ui/driver_ui/data.csv"

        with open(self.path_file) as f:
            self.data = [tuple(line) for line in csv.reader(f)]
        
            
        #for line in open(path_file):
            #= float(line.split(",")[0]
            #self.j = float(line.split(",")[1]
    def timer_callback(self):
        self.msg = Vector3()
        self.msg.x = float(self.data[self.i][0])
        self.msg.y = float(self.data[self.i][1])
        self.msg.z = 0.0

        #print(self.msg.x)
        #print(self.msg.y)
        #print(self.msg.z)

        self.publisher_.publish(self.msg)
        self.get_logger().info(f"x velocity = {self.msg.x}, y velocity = {self.msg.y}, z velocity = {self.msg.z}")
        self.i += 1
        
        #msg.data = 'Hello World: %d' % self.i
        #msg.x = msg.y = msg.z = self.i


        #for line in open(path_file):
            #self.i = float(line.split(",")[0]
            #self.j = float(line.split(",")[1]

#ros2 run pypubsub

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
