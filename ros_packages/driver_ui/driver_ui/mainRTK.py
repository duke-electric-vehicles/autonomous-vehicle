import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3

import pygame
import math
import random
from random import randint

"""
CONSTANTS
"""

BLACK = (0, 0, 0)
GRAY = (160, 160, 160)
BLUE = (51, 51, 255)
GREEN = (51, 255, 153)
RED = (255, 20, 40, 125)
transparent_RED = (255, 20, 40, 125)
transparent_Green = GREEN = (51, 255, 153, 125)

# UPDATE_DELAY = 1 # How often should the UI redraw the path with a new current position
LINE_THICKNESS = 6 # How thick should the path line be

class DriverUI(Node):

    def __init__(self):
        super().__init__('driver_ui')
        self.subscription = self.create_subscription(
            Vector3,
            'rtk_pos',
            self.pos_callback,
            10)

        self.subscription  # prevent unused variable warning

        # Pygame init stuff

        """
        INPUT:
        path: Path points as coordinates in a CSV file
        current_car_position: Coordinates of the currents car position from the ROS publisher/subscriber
        current_car_rotation: Degrees from NORTH that the car GPS is currently pointed towards (like a compass)
        """

        
        self.points = []
        self.points.append((1.1, 0.0))

        self.current_car_position = random.choice(self.points) # set current car position to a random point for now
        self.current_car_rotation = 0 # set current car rotation to be pointed straight NORTH

        pygame.init()
        self.current_pos = (1.000,0.000)
        self.current_vel = (0.000,0.000)
        self.ideal_vel = (0.000,0.000)
        self.current_speed = 0.000

        self.xpos = 600
        self.ypos = 300

        self.screen = pygame.display.set_mode((1200, 600))

        timer_period = 1/60  # seconds per frame
        self.timer = self.create_timer(timer_period, self.update_display)
       
    def update_display(self):
        self.points.append((self.current_pos[0], self.current_pos[1]))
        self.draw_points(self.points, self.current_pos, self.current_vel, self.current_speed, self.ideal_vel, self.current_car_rotation, self.xpos, self.ypos)

        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    print("ESC was pressed. quitting...")
                    quit()

    def pos_callback(self, msg):
        WIDTH, HEIGHT = pygame.display.get_surface().get_size()
        self.screen.fill(GRAY)
        x_pos = msg.x
        y_pos = msg.y

        self.current_pos = (round(x_pos, 3), round(y_pos, 3))
        self.current_vel = (round(x_pos, 3) / 1.6, round(y_pos, 3) / 2)
        self.ideal_vel = (round(x_pos, 3) / 2.1, round(y_pos, 3) / 2.1)
        self.current_speed = round(((self.current_vel[0] ** 2) + (self.current_vel[1] ** 2)) ** 0.5, 3)
        
        
        self.xpos += randint(-10, 10)



 
    def draw_points(self, points, current_position, current_velocity, current_speed, ideal_velocity, rotation_from_north, xpos, ypos):
        screen = self.screen
        WIDTH, HEIGHT = pygame.display.get_surface().get_size()
        screen.fill(GRAY)
        

        # screen extras
        font = pygame.font.SysFont(None, 24)
        bigFont = pygame.font.SysFont('DS-DIGIB.TTF', 80)
        current_label = font.render('Current Position: ' + str(current_position), True, BLUE)
        current_speed = font.render(str(current_speed) + ' m/s', True, BLUE)
        current_velocity_label = font.render('Current Speed: ' + str(current_velocity), True, BLUE)
        ideal_speed_label = font.render('Ideal Speed: ' + str(ideal_velocity), True, BLUE)

        moveUp = False
        moveDown = False
        moveRight = False
        moveLeft = False
        

        #heatmap
        HEAT_BAR_IMAGE = pygame.Surface((250, 20))
        color = pygame.Color(255, 0, 0)
        color2 = pygame.Color(0, 0, 255)
        # Fill the image with a simple gradient.
        for x in range(round(HEAT_BAR_IMAGE.get_width() / 2)):
            for y in range(HEAT_BAR_IMAGE.get_height()):
                HEAT_BAR_IMAGE.set_at((x, y), color)
            
            if color.g < 254:
                color.g += 2
            if color.r > 1:
                color.r -= 2
        for x in range(round(HEAT_BAR_IMAGE.get_width() / 2), HEAT_BAR_IMAGE.get_width()):
            for y in range(HEAT_BAR_IMAGE.get_height()):
                HEAT_BAR_IMAGE.set_at((x, y), color)
            
            if color.r < 254:
                color.r += 2
            if color.g > 1:
                color.g -= 2
    
        

        heat_rect = HEAT_BAR_IMAGE.get_rect(topleft=(100, 400))
        #ticker
        f = pygame.font.SysFont("Times New Roman", 80)
        arrow = f.render(u'\u2193', True, (255,255,0))
        


        #pygame.draw.rect(screen, RED, [20,HEIGHT-60,60,40])
        #pygame.draw.rect(screen, GREEN, [80,HEIGHT-60,60,40])

        #pygame.draw.rect(screen, RED, [140,HEIGHT-60,60,40])
        #pygame.draw.rect(screen, GREEN, [200,HEIGHT-60,60,40])

        circ_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        
        #pygame.draw.rect(circ_surface, BLACK, (130, 175, 320, 200), 10)

        
        #pygame.draw.circle(circ_surface, transparent_Green, [900, 300], 100)


        screen.blit(circ_surface, (0,0))
        screen.blit(current_label, (20, 20))
        screen.blit(current_velocity_label, (20, 60))
        screen.blit(ideal_speed_label, (20, 100))
        screen.blit(current_speed, (20, 140))
        
        #heatmap and ticker
        screen.blit(pygame.transform.scale(HEAT_BAR_IMAGE, (1000, 80)), heat_rect, (0, 0, 1000, 600))
        #screen.blit(HEAT_BAR_IMAGE, heat_rect, (0, 0, 1000, 600))
        screen.blit(arrow, (self.xpos, self.ypos))

        pygame.display.update()

def main(args=None):

    rclpy.init(args=args)

    driver_ui = DriverUI()
    rclpy.spin(driver_ui)

    driver_ui.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':

    main()
