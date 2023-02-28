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
        #self.width = pygame.display.Info().current_w
        #self.height = pygame.display.Info().current_h

        self.width = 400
        self.height = 400

        self.screen = pygame.display.set_mode((self.width, self.height))
  
        self.clock = pygame.time.Clock()
        self.time_elapsed_milli = 0
        self.clock_status = False

        timer_period = 1/60  # seconds per frame

        self.init_ui(self.height, self.width)
        timer_period = 1/60  # seconds per frame
        self.screen.fill((0, 0, 0))
        self.timer = self.create_timer(timer_period, self.update_display)


    #Main frame loop
    def timer_callback(self) -> None:

        if self.clock_status:
            #reset screen/(surface in the future)
            self.screen.fill((0, 0, 0))
            
            self.time_elapsed_milli += self.clock.get_time()
            self.time_elapsed_seconds = (self.time_elapsed_milli / 1000)
            self.time_elapsed_seconds = round(self.time_elapsed_seconds, 1)
        
            FONT = pygame.font.SysFont("Sans", 20)

            message = 'Seconds since enter: ' + str(self.time_elapsed_seconds)
            self.screen.blit(FONT.render(message, True, (120, 120, 120)), (20, 20))

        return

    def update_display(self) -> None:
        
        #timer init
        self.clock.tick(60)
        self.timer_callback()
        self.jackson_test()

        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:

                #pause timer
                if event.key == pygame.K_p:
                    self.timer_event = 'paused'
                    self.clock_status = False

                if event.key == pygame.K_o:
                    self.timer_event = 'unpaused'
                    self.clock_status = True

                #exit condition by pressing 'esc' button
                if event.key == pygame.K_ESCAPE:
                    print("ESC was pressed. quitting...")
                    quit() 

        pygame.display.update()

    def jackson_test(self) -> None:
        FONT = pygame.font.SysFont("Sans", 20)
        message = 'Jackson Test asdfasdasdfasdff'
        self.screen.blit(FONT.render(message, True, (120, 120, 120)), (20, 40))

    #unchanged surfaces on the screen
    def init_ui(self, height, width):
        #test ui for now
        
        return

    #need to make static background resizeable
    #dependent on self.width, self.height
    #already in lat/long form from publisher.py
    def position_callback(self, msg):




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
