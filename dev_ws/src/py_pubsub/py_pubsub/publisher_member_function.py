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

from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.client.loggers.json_logger import JSONLogger
from sbp.navigation import SBP_MSG_BASELINE_NED, MsgBaselineNED
import argparse

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.i = 0
        self.parser = argparse.ArgumentParser(
            description="Swift Navigation SBP Example.")
        self.parser.add_argument(
            "-p",
            "--port",
            default=['/dev/ttyACM1'],
            nargs=1,
            help="specify the serial port to use.")
        self.args = self.parser.parse_args()
        self.get_rtk_data()

    def get_rtk_data(self):
        data = String()

        # Open a connection to Piksi using the default baud rate (1Mbaud)
        with PySerialDriver(self.args.port[0], baud=1000000) as driver:
            with Handler(Framer(driver.read, None, verbose=True)) as source:
                try:
                    for msg, metadata in source.filter(SBP_MSG_BASELINE_NED):
                        # Print out the N, E, D coordinates of the baseline
                        data.data = "%.4f,%.4f,%.4f" % (msg.n * 1e-3, msg.e * 1e-3, msg.d * 1e-3)
                        self.publisher_.publish(data)
                        self.get_logger().info('Publishing: "%s"' % data.data)
                        self.i += 1

                except KeyboardInterrupt:
                    self.destroy_node()
                    rclpy.shutdown()


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
