import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3

import pygame
import math
import time
import datetime 
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
        self.points.append((0.0, 0.0, 219))
        self.index = 0

        self.current_car_position = random.choice(self.points) # set current car position to a random point for now
        self.current_car_rotation = 0 # set current car rotation to be pointed straight NORTH

        pygame.init()
        self.current_pos = (0,0,219)
        self.current_vel = (0,0,0)
        self.total_distance = 0
        self.current_speed = 0
        
        self.time = time.time()
        self.time2 = time.time()
        self.time_delta = 0

        self.xpos = 550
        self.ypos = 280

        self.xpos2 = 100
        self.ypos2 = 420

        self.screen = pygame.display.set_mode((1200, 600))
        #self.screen = pygame.display.set_mode((0,0), pygame.FULLSCREEN)

        timer_period = 1/60  # seconds per frame
        self.timer = self.create_timer(timer_period, self.update_display)
       
    def update_display(self):
        self.points.append((self.current_pos[0], self.current_pos[1], self.current_pos[2]))
        self.index += 1
        self.time_delta = self.time2 - self.time
        self.draw_points(self.time_delta, self.points, self.current_pos, self.current_vel, self.current_speed, self.total_distance, self.current_car_rotation, self.xpos, self.ypos)

        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    print("ESC was pressed. quitting...")
                    quit()

    def pos_callback(self, msg):
        WIDTH, HEIGHT = pygame.display.get_surface().get_size()
        self.screen.fill(BLACK)
        x_pos = msg.x
        y_pos = msg.y
        z_pos = msg.z

        self.current_pos = (x_pos, y_pos, z_pos)

        self.current_vel = (x_pos / 1.6, y_pos / 2, z_pos)

        #euclidean distance (x,y,z)
        self.distance_delta = round(((((self.current_pos[0] - self.points[self.index - 1][0]) ** 2) + ((self.current_pos[1] - self.points[self.index - 1][1]) ** 2) + ((self.current_pos[2] - self.points[self.index - 1][2]) ** 2)) ** 0.5) * 0.000621371, 6)
        self.total_distance += round(self.distance_delta, 6)

        #meters to mile conversion
        print(self.total_distance)

        #arrow from total distance
        self.xpos2 += self.total_distance * 100

        #mm/s to mph
        self.current_speed = (((self.current_vel[0] ** 2) + (self.current_vel[1] ** 2) + (self.current_vel[2] ** 2)) ** 0.5) * .00223694
        
        self.xpos += randint(-10, 10)
        
        self.time2 = time.time()




 
    def draw_points(self, timedelta, points, current_position, current_velocity, current_speed, total_distance, rotation_from_north, xpos, ypos):
        screen = self.screen
        WIDTH, HEIGHT = pygame.display.get_surface().get_size()
        screen.fill(BLACK)
        

        # screen extras
        font = pygame.font.SysFont(None, 24)

        bigFont = pygame.font.Font('/opt/ros/dev_ws/src/driver_ui/driver_ui/DS-DIGIB.TTF', 24)

        current_label = bigFont.render('Current Position: ' + str(current_position) + ' meters', True, GREEN)
        current_speed = bigFont.render(str(current_speed) + ' mph', True, GREEN)
        current_velocity_label = bigFont.render('Current Speed: ' + str(current_velocity), True, GREEN)
        total_distance_label = bigFont.render('Total Distance: ' + str(total_distance) + ' miles', True, GREEN)
        total_time_label = bigFont.render('Total Time: ' + str(datetime.timedelta(seconds = timedelta)).split(".")[0], True, GREEN)
        

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
    
        

        heat_rect = HEAT_BAR_IMAGE.get_rect(topleft=(100, 350))
        #ticker
        f = pygame.font.Font("/opt/ros/dev_ws/src/driver_ui/driver_ui/times.ttf", 60, bold = True)
        arrow = f.render(u'\u25BC', True, BLUE)

        #scale for distance
        circ_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        
        pygame.draw.line(circ_surface, RED, [100, 465], [100, 500], 5)
        pygame.draw.line(circ_surface, RED, [200, 465], [200, 500], 5)
        pygame.draw.line(circ_surface, RED, [300, 465], [300, 500], 5)
        pygame.draw.line(circ_surface, RED, [400, 465], [400, 500], 5)
        pygame.draw.line(circ_surface, RED, [500, 465], [500, 500], 5)
        pygame.draw.line(circ_surface, RED, [600, 465], [600, 500], 5)
        pygame.draw.line(circ_surface, RED, [700, 465], [700, 500], 5)
        pygame.draw.line(circ_surface, RED, [800, 465], [800, 500], 5)
        pygame.draw.line(circ_surface, RED, [900, 465], [900, 500], 5)
        pygame.draw.line(circ_surface, RED, [1000, 465], [1000, 500], 5)
        pygame.draw.line(circ_surface, RED, [1100, 465], [1100, 500], 5)
        pygame.draw.line(circ_surface, RED, [100, 500], [1100, 500], 5)

        #ticker for distance

        f = pygame.font.Font("/opt/ros/dev_ws/src/driver_ui/driver_ui/times.ttf", 60, bold = True)
        arrow2 = f.render(u'\u25BC', True, BLUE)




        #pygame.draw.polygon(screen, (0, 0, 0), ((0, 100), (0, 200), (200, 200), (200, 300), (300, 150), (200, 0), (200, 100)))
        
        

        #pygame.draw.rect(screen, RED, [20,HEIGHT-60,60,40])
        #pygame.draw.rect(screen, GREEN, [80,HEIGHT-60,60,40])

        #pygame.draw.rect(screen, RED, [140,HEIGHT-60,60,40])
        #pygame.draw.rect(screen, GREEN, [200,HEIGHT-60,60,40])

        
        
        
        #pygame.draw.rect(circ_surface, BLACK, (130, 175, 320, 200), 10)

        
        #pygame.draw.circle(circ_surface, transparent_Green, [900, 300], 100)


        screen.blit(circ_surface, (0,0))
        screen.blit(current_label, (20, 20))
        screen.blit(current_velocity_label, (20, 60))
        screen.blit(total_distance_label, (20, 100))
        screen.blit(current_speed, (20, 140))
        screen.blit(total_time_label, (20, 180))
        
        #heatmap and ticker
        screen.blit(pygame.transform.scale(HEAT_BAR_IMAGE, (1000, 60)), heat_rect, (0, 0, 1000, 600))
        #screen.blit(HEAT_BAR_IMAGE, heat_rect, (0, 0, 1000, 600))
        screen.blit(arrow, (self.xpos, self.ypos))
        screen.blit(arrow, (self.xpos2, self.ypos2))

        pygame.display.update()

def main(args=None):

    rclpy.init(args=args)

    driver_ui = DriverUI()
    rclpy.spin(driver_ui)

    driver_ui.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':

    main()
