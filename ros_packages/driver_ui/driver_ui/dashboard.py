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
position = (0, 0, 0)

#speed (km/h)
speed = 0

#power (watt)
power = 0

#time (seconds)
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

        timer_period = 1/60  # seconds per frame

        self.init_ui(self.height, self.width)

        self.timer = self.create_timer(timer_period, self.update_display)

    #Main frame loop
    def update_display(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                #self.init_ui(self.screen, self.height, self.width)
                #exit condition by pressing 'esc' button
                if event.key == pygame.K_ESCAPE:
                    print("ESC was pressed. quitting...")
                    quit()  

    #unchanged surfaces on the screen
    def init_ui(self, height, width):
        #test ui for now
        self.screen.fill((160, 160, 160))
        logo = pygame.image.load("/opt/ros/dev_ws/src/driver_ui/driver_ui/logo.png") 
        self.screen.blit(logo, (875, 35))  
        pygame.display.update()
        

    #need to make static background resizeable
    #dependent on self.width, self.height
    def position_callback(self, msg):
        
        x_pos = msg.x
        y_pos = msg.y
        z_pos = msg.z
        
        global position

        #tuple cur_pos
        position = (x_pos, y_pos, z_pos)


        return

    def speed_callback(self, msg) -> None:
        

        return
    
    def distance_callback(self, msg) -> None:
        return
    
    def power_callback(self, msg) -> None:
        return 

    def amps_callback(self, msg) -> None:
        return
    
    def voltage_callback(self, msg) -> None:
        return

    def joules_callback(self, msg) -> None:
        return

def main(args = None):

        rclpy.init(args=args)

        driver_ui = DriverUI()
        rclpy.spin(driver_ui)

        driver_ui.destroy_node()

        rclpy.shutdown()    


if __name__ == '__main__':

    main()
