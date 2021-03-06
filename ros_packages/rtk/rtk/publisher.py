# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Vector3

from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.client.loggers.json_logger import JSONLogger
from sbp.navigation import SBP_MSG_VEL_ECEF, SBP_MSG_POS_ECEF
import argparse

class RtkPublisher(Node):
    def __init__(self):
        super().__init__('rtk_publisher')
        self.pos_publisher = self.create_publisher(Vector3, 'rtk_pos', 10)
        self.vel_publisher = self.create_publisher(Vector3, 'rtk_vel', 10)

        self.parser = argparse.ArgumentParser(
            description="RTK Velocity Publisher")
        self.parser.add_argument(
            "-p",
            "--port",
            default=['/dev/ttyACM1'],
            nargs=1,
            help="specify the serial port to use.")
        self.args = self.parser.parse_args()
        self.driver = PySerialDriver(self.args.port[0], baud=1000000)

        self.handler = Handler(Framer(self.driver.read, None, verbose=True))
        self.handler.add_callback(self.pos_pub, msg_type=SBP_MSG_POS_ECEF)
        self.handler.add_callback(self.vel_pub, msg_type=SBP_MSG_VEL_ECEF)

        self.handler.start()

    def pos_pub(self, msg, **metadata):
        pos = Vector3()
        pos.x = msg.x
        pos.y = msg.y
        pos.z = msg.z
        self.pos_publisher.publish(pos)
        dataStr = "%.4f,%.4f,%.4f" % (msg.x * 1e-3, msg.y * 1e-3, msg.z * 1e-3)
        self.get_logger().info('Publishing Position: "%s"' % dataStr)

    def vel_pub(self, msg, **metadata):
        vel = Vector3()
        vel.x = msg.x * 1e-3
        vel.y = msg.y * 1e-3
        vel.z = msg.z * 1e-3
        self.vel_publisher.publish(vel)
        dataStr = "%.4f,%.4f,%.4f" % (msg.x * 1e-3, msg.y * 1e-3, msg.z * 1e-3)
        self.get_logger().info('Publishing Velocity: "%s"' % dataStr)


def main(args=None):
    rclpy.init(args=args)

    rtk_publisher = RtkPublisher()

    rclpy.spin(rtk_publisher)

    rtk_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
