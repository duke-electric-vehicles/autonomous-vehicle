import time
import random
from datetime import datetime
import pygame
import math
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoint
import rclpy
from rclpy.node import Node
import threading
import cv2
import numpy as np
import sys
from std_msgs.msg import Float64

#TODO
#remove threading to see if subscriber works

class DriverUI(Node):

    def create_button(self, text, x, y, w, h, color, text_color):
        button = pygame.Rect(x, y, w, h)
        pygame.draw.rect(self.screen, color, button)
        button_text = self.font_small.render(text, True, text_color)
        text_rect = button_text.get_rect(center=(x + w // 2, y + h // 2))
        self.screen.blit(button_text, text_rect)
        return button

    def __init__(self):
        super().__init__("driver_ui")

        #data sim
        self.subscriber1 = self.create_subscription(Float64, "pdb_voltage", self.voltage_callback, 10)
        self.subscriber2 = self.create_subscription(GeoPoint, "gps_data_sim", self.position_callback_speed, 10)
        # rtk
        
        self.subscription = self.create_subscription(GeoPoint, "rtk_pos", self.position_callback, 10)
        
        self.subscription = self.create_subscription(Vector3, "rtk_vel", self.speed_callback, 10)

        self.subscription = self.create_subscription(Float64, "pdb_current", self.current_callback, 10)
        
        
        pygame.init()

        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.GRAY = (192, 192, 192)
        self.BLUE = (0, 0, 255)
        self.GREEN = (0, 255, 0)
        self.RED = (255, 0, 0)

        self.SCREEN_WIDTH = 480
        self.SCREEN_HEIGHT = 800

        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT))
        # self.screen = pygame.display.set_mode((0,0), pygame.FULLSCREEN)

        pygame.display.set_caption("Driver Dashboard")

        self.font_small = pygame.font.SysFont("Calibri", 25, True, False)
        self.font_large = pygame.font.SysFont("Calibri", 50, True, False)
        self.font_xlarge = pygame.font.SysFont("Calibri", 85, True, False)
        self.font_gauge_numbers = pygame.font.SysFont("Calibri", 20, True, False) 

        # self.font_small = pygame.font.SysFont("Futura", 25, True, False)
        # self.font_large = pygame.font.SysFont("Futura", 50, True, False)
        # self.font_xlarge = pygame.font.SysFont("Futura", 75, True, False)
        # self.font_gauge_numbers = pygame.font.SysFont("Futura", 20, True, False)

        self.lat = 0
        self.lon = 0
        self.speed = 0
        self.distance = 0
        self.start_time = None
        self.stop_time = None
        self.running = False
        self.prev_lat = None
        self.prev_lon = None
        self.prev_timestamp = None
        self.cumulative_distance = 0.0
        self.speed_values = []
        self.speed_buffer_size = 5
        #self.show_camera = False
        #self.camera_thread = None
        self.voltage = 0.0
        self.current = 0.0
        #self.start_ros_thread()
         # Number of speed values to average
        timer_period = 1/60
        self.timer = self.create_timer(timer_period, self.run)

    # def generate_random_data(self):
    #     self.lat += random.uniform(-0.0005, 0.0005)
    #     self.lon += random.uniform(-0.0005, 0.0005)
    #     self.speed += random.uniform(-0.5, 0.5)

    def position_callback_speed(self, msg):
        if self.prev_lat is not None and self.prev_lon is not None:
            current_lat = msg.latitude
            current_lon = msg.longitude
            print(current_lon)
            current_timestamp = datetime.now()

            self.distance = self.calculate_distance(
                self.prev_lat, self.prev_lon, current_lat, current_lon
            )
            time_diff = (current_timestamp - self.prev_timestamp).total_seconds()
            self.cumulative_distance += self.distance

            if time_diff > 0:
                speed_kmh = (self.distance / time_diff) * 3600  # Speed in km/h
                speed_mph = speed_kmh * 0.621371  # Convert to mph

                # Store the new speed value and remove the oldest if the buffer is full
                self.speed_values.append(speed_mph)
                if len(self.speed_values) > self.speed_buffer_size:
                    self.speed_values.pop(0)

                # Calculate the average speed
                self.speed = sum(self.speed_values) / len(self.speed_values)

        self.prev_lat = msg.latitude
        self.prev_lon = msg.longitude
        self.prev_timestamp = datetime.now()

        self.lat = msg.latitude
        self.lon = msg.longitude

    def position_callback(self, msg):
        if self.prev_lat is not None and self.prev_lon is not None:
                current_lat = msg.latitude
                current_lon = msg.longitude
                print(current_lat)
                print(current_lon)
                current_timestamp = datetime.now()

                self.distance = self.calculate_distance(
                    self.prev_lat, self.prev_lon, current_lat, current_lon
                )

                self.cumulative_distance += self.distance
        self.prev_lat = msg.latitude
        self.prev_lon = msg.longitude
        self.prev_timestamp = datetime.now()

        self.lat = msg.latitude
        self.lon = msg.longitude

    def speed_callback(self, msg):
        xVel = msg.x
        yVel = msg.y
        zVel = msg.z
        
        magnitude = ((xVel ** 2) + (yVel ** 2)) ** 0.5

        self.speed_values.append(magnitude)

        if len(self.speed_values) > self.speed_buffer_size:
            self.speed_values.pop(0)

        self.speed = (sum(self.speed_values) / len(self.speed_values)) * 2.23694

    def current_callback(self, msg):
        self.current = msg.data
    
    def voltage_callback(self, msg):
        self.voltage = msg.data
        print(self.voltage)

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        radius_earth = 6371  # km
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (
            math.sin(dlat / 2) * math.sin(dlat / 2)
            + math.cos(math.radians(lat1))
            * math.cos(math.radians(lat2))
            * math.sin(dlon / 2)
            * math.sin(dlon / 2)
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        d = radius_earth * c
        return d

    def draw_text(self, text, font, color, x, y):
        text_surface = font.render(text, True, color)
        text_rect = text_surface.get_rect()
        text_rect.topleft = (x, y)
        self.screen.blit(text_surface, text_rect)

    def draw_progress_bar(self, value, max_value):
        bar_rect = pygame.Rect(50, 550, 380, 50)
        pygame.draw.rect(self.screen, self.GRAY, bar_rect, 5)
        progress_rect = pygame.Rect(
            55, 555, (self.SCREEN_WIDTH - 110) * (value / max_value), 40
        )
        pygame.draw.rect(self.screen, self.GREEN, progress_rect)

    def draw_cumulative_distance(self):
        distance_y = 625
        self.draw_text(
            f"Cumulative Distance: {self.cumulative_distance * 0.621371:.3f} miles",
            self.font_small,
            self.WHITE,
            100,
            distance_y,
        )

    def draw_gauge(self, x, y, radius, value, max_value, color, font):
        pygame.draw.circle(self.screen, color, (x, y), radius, 2)
        text = self.font_xlarge.render(f"{value:.2f}", True, color)
        text_rect = text.get_rect(center=(x, y))
        self.screen.blit(text, text_rect)

        # Draw numbers around the gauge
        num_interval = max_value // 20  # Set the interval for displaying numbers
        num_radius = radius - 15  # Adjust the position of the numbers
        for num in range(0, max_value + 1, num_interval):
            angle = 270 * (num / max_value) - 135
            rad_angle = math.radians(angle)
            num_x = x + num_radius * math.cos(rad_angle)
            num_y = y + num_radius * math.sin(rad_angle)
            num_text = self.font_gauge_numbers.render(str(num), True, color)
            num_rect = num_text.get_rect(center=(num_x, num_y))
            self.screen.blit(num_text, num_rect)


    def draw_needle(self, x, y, radius, value, max_value, color):
        angle = 270 * (value / max_value) - 135
        rad_angle = math.radians(angle)
        end_x = x + radius * math.cos(rad_angle)
        end_y = y + radius * math.sin(rad_angle)
        pygame.draw.line(self.screen, color, (x, y), (end_x, end_y), 2)
    
    def draw_center_circle(self, x, y, radius, color):
        pygame.draw.circle(self.screen, color, (x, y), radius)

    def run(self):
        #self.initialize_camera()
        self.screen.fill(self.BLACK)

        start_button = self.create_button("Start", 0, 700, 160, 100, self.BLUE, self.WHITE)
        stop_button = self.create_button("Stop", 160, 700, 160, 100, self.RED, self.WHITE)
        reset_button = self.create_button("Reset", 320, 700, 160, 100, self.GREEN, self.WHITE)

        self.draw_text("Driver Dashboard", self.font_large, self.WHITE, 100, 25)

        stopwatch_y = 100
        if self.running:
            elapsed_time = time.time() - self.start_time
        else:
            elapsed_time = (
                self.stop_time - self.start_time
                if self.start_time is not None and self.stop_time is not None
                else 0
            )
        minutes = int(elapsed_time // 60)
        seconds = int(elapsed_time % 60)
        milliseconds = int((elapsed_time % 1) * 1000)
        stopwatch_text = "Time: " + f"{minutes:02d}:{seconds:02d}:{milliseconds:03d}"
        self.draw_text(stopwatch_text, self.font_large, self.WHITE, 120, stopwatch_y)
        self.draw_text("Lat: " + f"{self.lat:.5f}", self.font_large, self.WHITE, 40, stopwatch_y + 70)
        self.draw_text("Lon: " + f"{self.lon:.5f}", self.font_large, self.WHITE, 260, stopwatch_y + 70)

        self.draw_text("V: " + f"{self.voltage:.2f}", self.font_large, self.WHITE, 10, 385)
        self.draw_text("I: " + f"{self.current:.2f}", self.font_large, self.WHITE, 370, 385)




        # speed_y = 300
        # self.draw_text(
        #     f"{self.speed:.2f} MPH", self.font_xlarge, self.WHITE, 50, speed_y
        # )

                    # ...

        # Speed Gauge
        gauge_x = self.SCREEN_WIDTH // 2
        gauge_y = 400
        gauge_radius = 110
        max_speed_value = 20  # Set the maximum speed value for the gauge
        
        self.draw_needle(gauge_x, gauge_y, gauge_radius, self.speed, max_speed_value, self.RED)

        center_radius = 75
        self.draw_center_circle(gauge_x, gauge_y, center_radius, self.BLACK)
        self.draw_gauge(gauge_x, gauge_y, gauge_radius, self.speed, max_speed_value, self.WHITE, self.font_large)

        self.draw_cumulative_distance()
        self.draw_progress_bar(self.cumulative_distance * 0.621371, 10)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                quit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s and not self.running:
                    if self.start_time is None:
                        self.start_time = time.time()
                    else:
                        self.start_time = (
                            time.time() - (self.stop_time - self.start_time)
                        )
                    self.running = True
                elif event.key == pygame.K_s and self.running:
                    self.stop_time = time.time()
                    self.running = False
                elif event.key == pygame.K_r:
                    self.start_time = None
                    self.stop_time = None
                    self.running = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = pygame.mouse.get_pos()
                if start_button.collidepoint(mouse_pos) and not self.running:
                    if self.start_time is None:
                        self.start_time = time.time()
                    else:
                        self.start_time = (
                            time.time() - (self.stop_time - self.start_time)
                        )
                    self.running = True
                elif stop_button.collidepoint(mouse_pos) and self.running:
                    self.stop_time = time.time()
                    self.running = False
                elif reset_button.collidepoint(mouse_pos):
                    self.start_time = None
                    self.stop_time = None
                    self.running = False

        pygame.display.update()

def main(args=None):
    rclpy.init(args=args)

    driver_ui = DriverUI()

    rclpy.spin(driver_ui)

    driver_ui.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()