import pygame
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from geographic_msgs.msg import GeoPoint

class DriverUI(Node):

    def __init__(self):
        super().__init__("driver_ui")

        self.subscription = self.create_subscription(
            GeoPoint, "gps_data_sim", self.position_callback, 10
        )

        self.subscription

        # Define some colors
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.GRAY = (192, 192, 192)
        self.BLUE = (0, 0, 255)
        self.GREEN = (0, 255, 0)
        self.RED = (255, 0, 0)

        # Set the dimensions of the screen
        self.SCREEN_WIDTH = 800
        self.SCREEN_HEIGHT = 600

        # Create the screen object
        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT))

        # Set the title of the window
        pygame.display.set_caption("Driver Dashboard")

        # Define some fonts
        self.font_small = pygame.font.SysFont('Calibri', 25, True, False)
        self.font_large = pygame.font.SysFont('Calibri', 50, True, False)

        # Define some variables
        self.lat = 0.0
        self.lon = 0.0
        self.speed = 0
        self.distance = 0
        self.start_time = None
        self.stop_time = None
        self.running = False

        # self.run()

    def position_callback(self, msg) -> None:
        self.lat = msg.latitude
        self.lon = msg.longitude

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        # Calculate the distance between two points on Earth using the Haversine formula
        radius_earth = 6371  # km

        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat / 2) * math.sin(dlat / 2) + math.cos(math.radians(lat1)) \
            * math.cos(math.radians(lat2)) * math.sin(dlon / 2) * math.sin(dlon / 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        d = radius_earth * c

        return d

    def draw_text(self, text, font, color, x, y):
        # Draw some text on the screen
        text_surface = font.render(text, True, color)
        text_rect = text_surface.get_rect()
        text_rect.topleft = (x, y)
        self.screen.blit(text_surface, text_rect)

    def draw_gauge(self, speed, max_speed):
        # Draw a speedometer-style gauge on the screen
        gauge_rect = pygame.Rect(400, 150, 200, 200)
        pygame.draw.arc(self.screen, self.WHITE, gauge_rect, math.pi * 1.1, math.pi * 1.9, 10)
        gauge_value = speed / max_speed * math.pi * 0.8 + math.pi * 0.1
        pygame.draw.arc(self.screen, self.BLUE, gauge_rect, math.pi * 1.1, gauge_value, 10)

        # Draw the speed as a number in the middle of the gauge
        # speed_text = self.font
        speed_text = self.font_small.render(f"{speed:02f}" + " MPH", True, self.WHITE)
        speed_rect = speed_text.get_rect()
        speed_rect.center = (500, 250)
        self.screen.blit(speed_text, speed_rect)

    def draw_progress_bar(self, value, max_value):
        # Draw a progress bar on the screen
        bar_rect = pygame.Rect(50, 300, 300, 50)
        pygame.draw.rect(self.screen, self.GRAY, bar_rect, 5)
        progress_rect = pygame.Rect(55, 305, (self.SCREEN_WIDTH - 110) * value / max_value, 40)
        pygame.draw.rect(self.screen, self.GREEN, progress_rect)

    def run(self):

        # Start the game loop
        while rclpy.ok():
            # Process ROS messages
            rclpy.spin_once(self, timeout_sec=0.1)
            # rclpy.spin_once(self, timeout = 0.1)
            # self.position_callback()
            # position_callback()
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    quit()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_s and not self.running:
                        if self.start_time is None:
                            self.start_time = time.time()
                        else:
                            self.start_time = time.time() - (self.stop_time - self.start_time)
                        running = True
                    elif event.key == pygame.K_s and not self.running:
                        if self.start_time is None:
                            self.start_time = time.time()
                        elif self.stop_time is not None:
                            self.start_time = time.time() - (self.stop_time - self.start_time)
                        self.running = True
                    elif event.key == pygame.K_s and self.running:
                        self.stop_time = time.time()
                        self.running = False
                    elif event.key == pygame.K_ESCAPE:
                        print("ESC was pressed. quitting...")
                        quit()

            # Clear the screen
            self.screen.fill(self.BLACK)

            # Draw the dashboard
            self.draw_text("Driver Dashboard", self.font_large, self.WHITE, 50, 25)

            # Draw the latitude and longitude
            self.draw_text(f"Latitude: {self.lat}", self.font_small, self.WHITE, 50, 100)
            self.draw_text(f"Longitude: {self.lon}", self.font_small, self.WHITE, 50, 150)

            # Draw the speedometer
            self.speed = self.speed + 0.001
            self.draw_gauge(self.speed, 20)

            # Draw the stopwatch
            if self.running:
                elapsed_time = time.time() - self.start_time
                minutes = int(elapsed_time // 60)
                seconds = int(elapsed_time % 60)
                milliseconds = int((elapsed_time % 1) * 1000)
                stopwatch_text = "Time: " + f"{minutes:02d}:{seconds:02d}:{milliseconds:03d}"
                self.draw_text(stopwatch_text, self.font_small, self.WHITE, 400, 100)

                # Update the distance
                if self.start_time is not None and self.running:
                    current_distance = self.calculate_distance(self.lat, self.lon, self.lat + 0.001, self.lon + 0.001)
                    self.distance += current_distance

            else:
                if self.start_time is not None and self.stop_time is not None:
                    elapsed_time = self.stop_time - self.start_time
                elif self.start_time is not None:  # Add this condition
                    elapsed_time = time.time() - self.start_time
                else:
                    elapsed_time = 0

                minutes = int(elapsed_time // 60)
                seconds = int(elapsed_time % 60)
                milliseconds = int((elapsed_time % 1) * 1000)
                stopwatch_text = "Time: " + f"{minutes:02d}:{seconds:02d}:{milliseconds:03d}"
                self.draw_text(stopwatch_text, self.font_small, self.WHITE, 400, 100)


            # Draw the distance
           
            # Draw the distance
            self.draw_progress_bar(self.distance, 10)

            # Update the screen
            pygame.display.update()

        # Quit the game
        pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    pygame.init()

    driver_ui = DriverUI()
    driver_ui.run()

    rclpy.shutdown()

if __name__ == "__main__":
    main()


import pygame
import time
import math
import random
# from geometry_msgs.msg import Vector3
# from geographic_msgs.msg import GeoPoint


# dummy screen
# class DriverUI:
#     def __init__(self):
#         pygame.init()

#         self.BLACK = (0, 0, 0)
#         self.WHITE = (255, 255, 255)
#         self.GRAY = (192, 192, 192)
#         self.BLUE = (0, 0, 255)
#         self.GREEN = (0, 255, 0)
#         self.RED = (255, 0, 0)

#         self.SCREEN_WIDTH = 480
#         self.SCREEN_HEIGHT = 800

#         self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT))

#         pygame.display.set_caption("Driver Dashboard")

#         self.font_small = pygame.font.SysFont("Calibri", 25, True, False)
#         self.font_large = pygame.font.SysFont("Calibri", 50, True, False)
#         self.font_xlarge = pygame.font.SysFont("Calibri", 75, True, False)

#         self.lat = 0
#         self.lon = 0
#         self.speed = 0
#         self.distance = 0
#         self.start_time = None
#         self.stop_time = None
#         self.running = False

#     def generate_random_data(self):
#         self.lat += random.uniform(-0.0005, 0.0005)
#         self.lon += random.uniform(-0.0005, 0.0005)
#         self.speed += random.uniform(-0.5, 0.5)

#     def calculate_distance(self, lat1, lon1, lat2, lon2):
#         radius_earth = 6371  # km
#         dlat = math.radians(lat2 - lat1)
#         dlon = math.radians(lon2 - lon1)
#         a = (
#             math.sin(dlat / 2) * math.sin(dlat / 2)
#             + math.cos(math.radians(lat1))
#             * math.cos(math.radians(lat2))
#             * math.sin(dlon / 2)
#             * math.sin(dlon / 2)
#         )
#         c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
#         d = radius_earth * c
#         return d

#     def draw_text(self, text, font, color, x, y):
#         text_surface = font.render(text, True, color)
#         text_rect = text_surface.get_rect()
#         text_rect.topleft = (x, y)
#         self.screen.blit(text_surface, text_rect)

#     def draw_progress_bar(self, value, max_value):
#         bar_rect = pygame.Rect(50, 550, 380, 50)
#         pygame.draw.rect(self.screen, self.GRAY, bar_rect, 5)
#         progress_rect = pygame.Rect(
#             55, 555, (self.SCREEN_WIDTH - 110) * value / max_value, 40
#         )
#         pygame.draw.rect(self.screen, self.GREEN, progress_rect)

#     def run(self):
#         done = False
#         while not done:
#             for event in pygame.event.get():
#                 if event.type == pygame.QUIT:
#                     done = True
#                 elif event.type == pygame.KEYDOWN:
#                     if event.key == pygame.K_s and not self.running:
#                         if self.start_time is None:
#                             self.start_time = time.time()
#                         else:
#                             self.start_time = (
#                                 time.time() - (self.stop_time - self.start_time)
#                             )
#                         self.running = True
#                     elif event.key == pygame.K_s and self.running:
#                         self.stop_time = time.time()
#                         self.running = False
#                     elif event.key == pygame.K_r:
#                         self.start_time = None
#                         self.stop_time = None
#                         self.running = False
#                         self.distance = 0
#                     elif event.key == pygame.K_ESCAPE:
#                         print("ESC was pressed. quitting...")
#                         quit()

#             self.screen.fill(self.BLACK)

#             self.generate_random_data()

#             self.draw_text("Driver Dashboard", self.font_large, self.WHITE, 50, 25)

#             stopwatch_y = 100
#             if self.running:
#                 elapsed_time = time.time() - self.start_time
#             else:
#                 elapsed_time = (
#                     self.stop_time - self.start_time
#                     if self.start_time is not None and self.stop_time is not None
#                     else 0
#                 )
#             minutes = int(elapsed_time // 60)
#             seconds = int(elapsed_time % 60)
#             milliseconds = int((elapsed_time % 1) * 1000)
#             stopwatch_text = f"{minutes:02d}:{seconds:02d}:{milliseconds:03d}"
#             self.draw_text(stopwatch_text, self.font_large, self.WHITE, 120, stopwatch_y)

#             speed_y = 300
#             self.draw_text(
#                 f"{self.speed:.2f} MPH", self.font_xlarge, self.WHITE, 50, speed_y
#             )

#             if self.running:
#                 current_distance = self.calculate_distance(
#                     self.lat, self.lon, self.lat + 0.001, self.lon + 0.001
#                 )
#                 self.distance += current_distance

#             self.draw_progress_bar(self.distance, 10000)

#             pygame.display.flip()

#         pygame.quit()


# if __name__ == "__main__":
#     ui = DriverUI()
#     ui.run()
