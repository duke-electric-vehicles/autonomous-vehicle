#LIBRARY IMPORTS
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import pygame
import time
import datetime 
from random import randint
import sys

#GLOBAL CONSTS

#pos tuple (x, y, z)
global position 
position = (0, 0, 0)

#speed (km/h)
global speed
speed = 0

#power (watt)
global power
power = 0

#time (seconds)
global timeLog
timeLog = 0

class DriverUI(Node):

    def __init__(self) -> None:
        super().__init__('driver_ui')

        self.subscription = self.create_subscription(
            Vector3,
            'rtk_pos',
            self.position_callback,
            10)

        self.subscription

        #initialize pygame
        pygame.init()

        #initialize pygame screen / get dimensions
        #self.width, self.height = pygame.display.get_surface().get_size()
        self.width = pygame.display.Info().current_w
        self.height = pygame.display.Info().current_h
        self.screen = pygame.display.set_mode((self.width, self.height))

        self.screen.fill((51, 51, 255))

        timer_period = 1/60  # seconds per frame
        self.timer = self.create_timer(timer_period, self.update_display)
        

    #need to make static background resizeable
    #dependent on self.width, self.height
    def speed_callback(self, msg) -> None:
        return
    
    def distance_callback(self, msg) -> None:
        return
    
    def position_callback(self, msg):
        self.screen.fill((0, 0, 255))
    
    def power_bacllback(self, msg) -> None:
        return 

    #Main frame loop
    def update_display(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:

                #exit condition by pressing 'esc' button
                if event.key == pygame.K_ESCAPE:
                    print("ESC was pressed. quitting...")
                    quit()


def main(args = None):
        
        print('Hello World')

        rclpy.init(args=args)

        driver_ui = DriverUI()
        rclpy.spin(driver_ui)

        driver_ui.destroy_node()

        rclpy.shutdown()    


if __name__ == '__main__':

    main()
