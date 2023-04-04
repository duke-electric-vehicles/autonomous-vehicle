import pygame
import time
import math
import sys

pygame.init()

# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GRAY = (192, 192, 192)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

# Set the dimensions of the screen
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

# Create the screen object
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.FULLSCREEN)

# Set the title of the window
pygame.display.set_caption("Driver Dashboard")

# Define some fonts
font_small = pygame.font.SysFont('Calibri', 25, True, False)
font_large = pygame.font.SysFont('Calibri', 50, True, False)

# Define some variables
lat = 37.7749
lon = -122.4194
speed = 0
distance = 0
start_time = None
stop_time = None
running = False

# def update_speed(speed):
#     speed = speed + 0.1
#     return speed

# Define some functions
def calculate_distance(lat1, lon1, lat2, lon2):
    # Calculate the distance between two points on Earth using the Haversine formula
    radius_earth = 6371 # km

    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2) * math.sin(dlat/2) + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2) * math.sin(dlon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = radius_earth * c

    return d

def draw_text(text, font, color, x, y):
    # Draw some text on the screen
    text_surface = font.render(text, True, color)
    text_rect = text_surface.get_rect()
    text_rect.topleft = (x, y)
    screen.blit(text_surface, text_rect)

def draw_gauge(speed, max_speed):
    # Draw a speedometer-style gauge on the screen
    gauge_rect = pygame.Rect(400, 150, 200, 200)
    pygame.draw.arc(screen, WHITE, gauge_rect, math.pi * 1.1, math.pi * 1.9, 10)
    gauge_value = speed / max_speed * math.pi * 0.8 + math.pi * 0.1
    pygame.draw.arc(screen, BLUE, gauge_rect, math.pi * 1.1, gauge_value, 10)

    # Draw the speed as a number in the middle of the gauge
    speed_text = font_small.render(f"{speed:02f}" + " MPH", True, WHITE)
    speed_rect = speed_text.get_rect()
    speed_rect.center = (500, 250)
    screen.blit(speed_text, speed_rect)


def draw_progress_bar(value, max_value):
    # Draw a progress bar on the screen
    bar_rect = pygame.Rect(50, 300, 300, 50)
    pygame.draw.rect(screen, GRAY, bar_rect, 5)
    progress_rect = pygame.Rect(55, 305, (SCREEN_WIDTH - 110) * value / max_value, 40)
    pygame.draw.rect(screen, GREEN, progress_rect)

def main():
    global lat, lon, speed, distance, start_time, stop_time, running

    # Start the game loop
    done = False
    while not done:

        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s and not running:
                    if start_time is None:
                        start_time = time.time()
                    else:
                        start_time = time.time() - (stop_time - start_time)
                    running = True
                elif event.key == pygame.K_s and running:
                    stop_time = time.time()
                    running = False
                elif event.key == pygame.K_r:
                    start_time = None
                    stop_time = None
                    running = False
                    distance = 0
                elif event.key == pygame.K_ESCAPE:
                    print("ESC was pressed. quitting...")
                    pygame.display.quit()
                    pygame.quit()
                    sys.exit()


        # Clear the screen
        screen.fill(BLACK)

        # Draw the dashboard
        draw_text("Driver Dashboard", font_large, WHITE, 50, 25)

        # Draw the latitude and longitude
        draw_text(f"Latitude: {lat}", font_small, WHITE, 50, 100)
        draw_text(f"Longitude: {lon}", font_small, WHITE, 50, 150)

        # Draw the speedometer
        speed = speed + 0.001
        draw_gauge(speed, 20)

        # Draw the stopwatch
        if running:
            elapsed_time = time.time() - start_time
            minutes = int(elapsed_time // 60)
            seconds = int(elapsed_time % 60)
            milliseconds = int((elapsed_time % 1) * 1000)
            stopwatch_text = "Time: " + f"{minutes:02d}:{seconds:02d}:{milliseconds:03d}"
            draw_text(stopwatch_text, font_small, WHITE, 400, 100)

            # Update the distance
            if start_time is not None and running:
                current_distance = calculate_distance(lat, lon, lat + 0.001, lon + 0.001)
                distance += current_distance

        else:
            if start_time is not None and stop_time is not None:
                elapsed_time = stop_time - start_time
                minutes = int(elapsed_time // 60)
                seconds = int(elapsed_time % 60)
                milliseconds = int((elapsed_time % 1) * 1000)
                stopwatch_text = "Time: " + f"{minutes:02d}:{seconds:02d}:{milliseconds:03d}"
                draw_text(stopwatch_text, font_small, WHITE, 400, 100)

        # Draw the distance
        draw_progress_bar(distance, 10000)

        # Update the screen
        pygame.display.flip()

    # Quit the game
    pygame.display.quit()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
