import time
import threading
import pygame
import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import String

pygame.init()
screen = pygame.display.set_mode((800, 600))

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    screen.fill((255, 255, 255))
    pygame.display.flip()