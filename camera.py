import time
import random
from datetime import datetime
import pygame
import math
import threading
import cv2
import numpy as np
import sys

class DriverUI():

    def create_button(self, text, x, y, w, h, color, text_color):
        button = pygame.Rect(x, y, w, h)
        pygame.draw.rect(self.screen, color, button)
        button_text = self.font_small.render(text, True, text_color)
        text_rect = button_text.get_rect(center=(x + w // 2, y + h // 2))
        self.screen.blit(button_text, text_rect)
        return button

    def __init__(self):
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
        self.show_camera = True
        self.camera_thread = None
        self.voltage = 0.0
        self.current = 0.0
    
    def initialize_camera(self):
        #raspberry pi index = 1
        self.cap = cv2.VideoCapture(1)

    def display_camera(self):
        ret, frame = self.cap.read()  # Read a frame from the camera
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert the frame to RGB
            frame = cv2.flip(frame, 1)  # Flip the frame horizontally
            #include all of the camera
            frame = cv2.resize(frame, (480, 480))
            frame = np.rot90(frame)  # Rotate the frame 90 degrees counter-clockwise
            frame = pygame.surfarray.make_surface(frame)  # Convert the frame to a Pygame surface
            #make outline on frame surf
            pygame.draw.rect(frame, (0, 0, 255), frame.get_rect(), 5)
            self.screen.blit(frame, (0, 0))  # Draw the frame on the screen

    def draw_text(self, text, font, color, x, y):
        text_surface = font.render(text, True, color)
        text_rect = text_surface.get_rect()
        text_rect.topleft = (x, y)
        self.screen.blit(text_surface, text_rect)

    def stop_camera_feed(self):
        if self.cap:
            self.cap.release()
        if self.camera_thread:
            self.camera_thread.join()
            self.camera_thread = None




    def run(self):
        self.initialize_camera()

        # start_button = self.create_button("Start", 0, 700, 160, 100, self.BLUE, self.WHITE)
        # stop_button = self.create_button("Stop", 160, 700, 160, 100, self.RED, self.WHITE)
        # reset_button = self.create_button("Reset", 320, 700, 160, 100, self.GREEN, self.WHITE)

        done = False
        while not done:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True
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
                    elif event.key == pygame.K_c:
                        self.show_camera = not self.show_camera
                        if self.show_camera:
                            self.camera_thread = threading.Thread(target=self.show_camera)
                            self.camera_thread.start()
                        else:
                            self.stop_camera_feed()
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

            if self.show_camera:
                cover_color = (0, 0, 0)
                cover_rect = pygame.Rect(0, 0, self.SCREEN_WIDTH, self.SCREEN_HEIGHT)
                
                #stopwatch routine
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
                stopwatch_text = f"{minutes:02d}:{seconds:02d}"
                self.draw_text(stopwatch_text, self.font_large, self.WHITE, 120, stopwatch_y)
                
                #testing purposes
                #self.speed = self.lon
                #camera routine
                pygame.draw.rect(self.screen, cover_color, cover_rect)
                self.draw_text("NO CAMERA DETECTED", self.font_large, self.RED, 50, 200)
                self.display_camera()
                self.draw_text(stopwatch_text, self.font_xlarge, self.WHITE, 135, 500)
                #self.draw_text("Scuffed Velocity", self.font_large, self.WHITE, 100, 600)
                #self.draw_text(str(self.speed), self.font_large, self.WHITE, 125, 650)

            start_button = self.create_button("Start", 0, 625, 160, 175, self.GREEN, self.WHITE)
            stop_button = self.create_button("Stop", 160, 625, 160, 175, self.RED, self.WHITE)
            reset_button = self.create_button("Reset", 320, 625, 160, 175, self.BLUE, self.WHITE)

            pygame.display.flip()

        self.cap.release()
        pygame.quit()

def main(args=None):

    driver_ui = DriverUI()
    driver_ui.run()

if __name__ == "__main__":
    main()
