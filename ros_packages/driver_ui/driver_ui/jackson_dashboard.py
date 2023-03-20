import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from geographic_msgs.msg import GeoPoint
import pygame
import math
import time

pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)

class DriverDashboard(Node):
    def __init__(self):
        super().__init__('driver_dashboard')

        self.lat = 0.0
        self.long = 0.0
        self.alt = 0.0

        self.subscription = self.create_subscription(
            GeoPoint,
            'gps_data_sim',
            self.position_callback,
            10)

    def position_callback(self, msg):
        self.lat = msg.latitude
        self.long = msg.longitude
        self.alt = msg.altitude

def draw_elapsed_time(screen, elapsed_time):
    font = pygame.font.Font(None, 36)
    text = font.render("Elapsed Time: {:.2f} s".format(elapsed_time), True, WHITE)
    screen.blit(text, (10, 10))

def main():
    rclpy.init()

    dashboard_node = DriverDashboard()

    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Driver Dashboard")

    clock = pygame.time.Clock()

    running = True
    show_elapsed_time = False
    start_time = 0.0
    elapsed_time = 0.0

    while running and not dashboard_node.get_executor().is_shutdown():
        screen.fill(BLACK)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s:
                    show_elapsed_time = True
                    start_time = time.time()
                elif event.key == pygame.K_r:
                    show_elapsed_time = False
                    elapsed_time = 0.0

        if show_elapsed_time:
            elapsed_time = time.time() - start_time
            draw_elapsed_time(screen, elapsed_time)

        # Add your drawing functions here

        pygame.display.flip()
        clock.tick(30)

    dashboard_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
