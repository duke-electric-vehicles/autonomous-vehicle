import pygame
import math
import pandas as pd

class Dashboard:
    def __init__(self, screen):
        self.screen = screen
        self.font_xlarge = pygame.font.SysFont('Arial', 100)
        self.font_med = pygame.font.SysFont('Arial', 50)
        self.font_gauge_numbers = pygame.font.SysFont('Arial', 27)
        self.lap_count = 0
        
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.RED = (255, 0, 0)
        self.GREEN = (0, 255, 0)
        self.BLUE = (0, 0, 255)

        self.font_small = pygame.font.SysFont("Calibri", 25, True, False)

    def draw_lap_count(self):
        lap_text = f"Lap: {self.lap_count}"
        text_surf = self.font_med.render(lap_text, True, self.WHITE)
        text_rect = text_surf.get_rect(topright=(1550, 10))  # Position at top left
        self.screen.blit(text_surf, text_rect)


    def draw_gauge(self, x, y, radius, value, max_value, color, font):
        pygame.draw.circle(self.screen, color, (x, y), radius, 2)
        text = self.font_xlarge.render(f"{value:.2f}", True, color)
        text_rect = text.get_rect(center=(x, y))
        self.screen.blit(text, text_rect)

        # Draw numbers around the gauge
        num_interval = max_value // 20  # Set the interval for displaying numbers
        num_radius = radius - 25  # Adjust the position of the numbers
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

    def draw_stopwatch(self, elapsed_time):
        mins, secs = divmod(elapsed_time // 1000, 60)
        milliseconds = elapsed_time % 1000 // 100
        time_format = '{:02d}:{:02d}.{:01d}'.format(mins, secs, milliseconds)
        text_surf = self.font_xlarge.render(time_format, True, self.WHITE)
        text_rect = text_surf.get_rect(center=(300, 300))  # Adjust position as needed
        self.screen.blit(text_surf, text_rect)

    def draw_buttons(self):
        # Define button properties
        button_height = 50
        button_width = 100
        corner_radius = 10
        
        # Adjust positions/sizes as needed
        self.start_button_rect = pygame.Rect(100, 500, button_width, button_height)
        draw_rounded_rect(self.screen, self.start_button_rect, self.GREEN, corner_radius)
        start_text = self.font_small.render('Start', True, self.BLACK)
        self.screen.blit(start_text, self.start_button_rect.topleft + pygame.math.Vector2(10, 10))
        
        self.stop_button_rect = pygame.Rect(250, 500, button_width, button_height)
        draw_rounded_rect(self.screen, self.stop_button_rect, self.RED, corner_radius)
        stop_text = self.font_small.render('Stop', True, self.BLACK)
        self.screen.blit(stop_text, self.stop_button_rect.topleft + pygame.math.Vector2(10, 10))
        
        self.reset_button_rect = pygame.Rect(400, 500, button_width, button_height)
        draw_rounded_rect(self.screen, self.reset_button_rect, self.BLUE, corner_radius)
        reset_text = self.font_small.render('Reset', True, self.BLACK)
        self.screen.blit(reset_text, self.reset_button_rect.topleft + pygame.math.Vector2(10, 10))
    
    def check_button_click(self, position):
        if self.start_button_rect.collidepoint(position):
            return 'start'
        elif self.stop_button_rect.collidepoint(position):
            return 'stop'
        elif self.reset_button_rect.collidepoint(position):
            return 'reset'
        return None
    
def draw_rounded_rect(surface, rect, color, corner_radius):
    '''Draws a rectangle with rounded corners on the given surface.'''
    if rect.width < 2 * corner_radius or rect.height < 2 * corner_radius:
        raise ValueError("The rectangle is too small for the given corner radius.")

    # Corner circles
    pygame.draw.circle(surface, color, (rect.left + corner_radius, rect.top + corner_radius), corner_radius)
    pygame.draw.circle(surface, color, (rect.right - corner_radius, rect.top + corner_radius), corner_radius)
    pygame.draw.circle(surface, color, (rect.left + corner_radius, rect.bottom - corner_radius), corner_radius)
    pygame.draw.circle(surface, color, (rect.right - corner_radius, rect.bottom - corner_radius), corner_radius)

    # Rectangle to cover the top and bottom sides
    pygame.draw.rect(surface, color, rect.inflate(0, -corner_radius * 2))

    # Rectangle to cover the left and right sides
    pygame.draw.rect(surface, color, rect.inflate(-corner_radius * 2, 0))


def normalize_coords(coords, screen_width, screen_height, margin=20, track_offset_x=1200):
    lat_min, lat_max = coords['Latitude'].min(), coords['Latitude'].max()
    long_min, long_max = coords['Longitude'].min(), coords['Longitude'].max()
    
    coords['norm_x'] = (((coords['Longitude'] - long_min) / (long_max - long_min)) * (screen_width/2 - 2*margin)) + margin + track_offset_x
    coords['norm_y'] = ((coords['Latitude'] - lat_max) / (lat_min - lat_max)) * (screen_height - 2*margin) + margin
    
    return coords

def draw_track(screen, coords, dashboard):
    for i, row in coords.iterrows():
        pygame.draw.circle(screen, dashboard.WHITE, (int(row['norm_x']), int(row['norm_y'])), 2)

def draw_live_coord(screen, coords, current_index, dashboard):
    row = coords.iloc[current_index]
    pygame.draw.circle(screen, dashboard.RED, (int(row['norm_x']), int(row['norm_y'])), 5)

def load_track_data(file_path):
    return pd.read_csv(file_path)


def main():
    pygame.init()
    pygame.display.set_caption('Driver Dashboard')
    window_surface = pygame.display.set_mode((1600, 600))
    dashboard = Dashboard(window_surface)
    background = pygame.Surface((1600, 600))
    background.fill(pygame.Color('#000000'))

    # Load and normalize track coordinates
    coords_df = load_track_data('/opt/ros/dev_ws/src/2024ui/2024ui/track_coords.csv')
    normalized_coords = normalize_coords(coords_df, 800, 600)
    current_index = 0

    is_running = True
    speed = 0
    max_speed_value = 20  # Set the maximum speed value for the gauge

    cumulative_distance = 0
    
    stopwatch_running = False
    start_time = 0
    end_time = 0
    elapsed_time = 0

    current_index_count = 0

    while is_running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                is_running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                clicked_button = dashboard.check_button_click(event.pos)
                if clicked_button == 'start':
                    if not stopwatch_running:
                        stopwatch_running = True
                        start_time = pygame.time.get_ticks() - elapsed_time
                elif clicked_button == 'stop':
                    stopwatch_running = False
                    elapsed_time = pygame.time.get_ticks() - start_time
                elif clicked_button == 'reset':
                    elapsed_time = 0
                    if stopwatch_running:
                        start_time = pygame.time.get_ticks()
        window_surface.blit(background, (0, 0))


        if stopwatch_running:
            elapsed_time = pygame.time.get_ticks() - start_time

        dashboard.draw_buttons()
        
        dashboard.draw_stopwatch(elapsed_time)

        # Drawing the track and live coordinate
        draw_track(window_surface, normalized_coords, dashboard)
        draw_live_coord(window_surface, normalized_coords, current_index, dashboard)
        current_index = (current_index + 1) % len(normalized_coords)

        gauge_x = 800
        gauge_y = 300
        gauge_radius = 250
        speed = (speed + 0.05) % 20
        max_speed_value = 20  # Set the maximum speed value for the gauge

        dashboard.draw_needle(
            gauge_x, gauge_y, gauge_radius, speed, max_speed_value, dashboard.RED
        )

        center_radius = 150
        dashboard.draw_center_circle(gauge_x, gauge_y, center_radius, dashboard.BLACK)
        dashboard.draw_gauge(
            gauge_x,
            gauge_y,
            gauge_radius,
            speed,
            max_speed_value,
            dashboard.WHITE,
            dashboard.font_xlarge,
        )

        current_index_count += 30

        lap_count = math.floor(current_index_count / len(normalized_coords)) + 1
        dashboard.lap_count = lap_count
        dashboard.draw_lap_count()

        pygame.display.update()


        current_index = (current_index + 30) % len(normalized_coords)

if __name__ == '__main__':
    main()