# LIBRARY IMPORTS
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from geographic_msgs.msg import GeoPoint
import pygame
import time
import datetime
from random import randint
import sys


class DriverUI(Node):
    def __init__(self) -> None:
        super().__init__("driver_ui")
        # pos subscriber service
        self.subscription = self.create_subscription(
            GeoPoint, "gps_data_sim", self.position_callback, 10
        )
        # vel subscriber service
        """
        self.subscription = self.create_subscription(
            Vector3,
            'rtk_vel',
            self.velocity_callback,
            10)   
        """

        self.subscription

        # initialize pygame
        pygame.init()


        # Global Data Vars
        self.alt = "0.0"
        self.lat = "0.0"
        self.long = "0.0"

        # initialize pygame screen / get dimensions
        # self.width, self.height = pygame.display.get_surface().get_size()
        self.width = pygame.display.Info().current_w
        self.height = pygame.display.Info().current_h

        #self.width = 800
        #self.height = 480

        self.width = 800
        self.height = 480
        self.screen = pygame.display.set_mode((self.width, self.height))

        # initialize clock
        self.clock = pygame.time.Clock()
        self.time_elapsed_seconds = 0
        self.time_elapsed_milli = 0
        self.clock_status = False 

        timer_period = 1 / 60  # seconds per frame
        self.screen.fill((0, 0, 0))

        self.timer = self.create_timer(timer_period, self.update_display)

    # Main frame loop
    def timer_callback(self) -> None:

        if self.clock_status:
            # reset screen/(surface in the future)

            self.time_elapsed_milli += self.clock.get_time()
            self.time_elapsed_seconds = self.time_elapsed_milli / 1000
            self.time_elapsed_seconds = round(self.time_elapsed_seconds, 1)
        
        return 

    def update_display(self) -> None:
        self.screen.fill((0, 0, 0))
        # timer fps init
        self.clock.tick(60)
        self.timer_callback()

        FONT = pygame.font.SysFont("Sans", 20)
        
        message = "Seconds since enter: " + str(self.time_elapsed_seconds)

        msg = FONT.render(message, True, (120, 120, 120))
        #msg = pygame.transform.rotate(msg, 90)
        
        long_msg = FONT.render(self.long, True, (120, 120, 120))
        #long_msg = pygame.transform.rotate(long_msg, 90)

        lat_msg = FONT.render(self.lat, True, (120, 120, 120))
        #lat_msg = pygame.transform.rotate(lat_msg, 90)

        alt_msg = FONT.render(self.alt, True, (120, 120, 120))
        #alt_msg = pygame.transform.rotate(alt_msg, 90)

        self.screen.blit(msg, (20, 20))
        self.screen.blit(long_msg, (40, 40))
        self.screen.blit(lat_msg, (60, 60))
        self.screen.blit(alt_msg, (80, 80))

        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                # pause timer
                if event.key == pygame.K_p:
                    self.timer_event = "paused"
                    self.clock_status = False
                # unpause timer
                if event.key == pygame.K_o:
                    self.timer_event = "unpaused"
                    self.clock_status = True
                # exit condition by pressing 'esc' button
                if event.key == pygame.K_ESCAPE:
                    print("ESC was pressed. quitting...")
                    quit()

            # if x is pressed quit
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_q):
                # exit if q is pressed
                pygame.quit()
                sys.exit()

        pygame.display.update()

        return 

    # surface outline of the screen
    def static_ui(self) -> None:
        # test ui for now
        self.ui_outline = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        
        return

    # need to make static background resizeable
    # dependent on self.width, self.height
    # already in lat/long form from publisher.py
    def position_callback(self, msg) -> None:
        self.alt = str(msg.altitude)
        self.lat = str(msg.latitude)
        self.long = str(msg.longitude)

        return

    def velocity_callback(self) -> None:
        return

    def distance_callback(self) -> None:
        return

    def power_callback(self) -> None:
        return

    def amps_callback(self) -> None:
        return

    def voltage_callback(self) -> None:
        return

    def joules_callback(self) -> None:
        return


def main(args=None):
    rclpy.init(args=args)

    driver_ui = DriverUI()
    rclpy.spin(driver_ui)

    driver_ui.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
