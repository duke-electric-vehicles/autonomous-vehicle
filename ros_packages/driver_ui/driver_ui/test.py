import pygame
import math
import random

"""
CONSTANTS
"""

BLACK = (0, 0, 0)
GRAY = (160, 160, 160)
BLUE = (51, 51, 255)
GREEN = (51, 255, 153)
RED = (255, 20, 40)

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

        path_file = "/opt/ros/dev_ws/src/driver_ui/driver_ui/data.csv"
        self.points = []

        for line in open(path_file):
            self.points.append((float(line.split(",")[0]), float(line.split(",")[1])))

        self.current_car_position = random.choice(self.points) # set current car position to a random point for now
        self.current_car_rotation = 0 # set current car rotation to be pointed straight NORTH

        pygame.init()

        self.screen = pygame.display.set_mode((1200, 600))

        timer_period = 1/60  # seconds per frame
        self.timer = self.create_timer(timer_period, self.update_display)
       
    def update_display(self):
        pygame.display.update()
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    print("ESC was pressed. quitting...")
                    quit()

    def pos_callback(self, msg):
        x_pos = msg.x
        y_pos = msg.y

        current_pos = (x_pos, y_pos)

        print(f"Current position: {current_pos}")

        self.draw_points(self.points, current_pos, self.current_car_rotation)

 
    def draw_points(self, points, current_position, rotation_from_north):
        screen = self.screen
        WIDTH, HEIGHT = pygame.display.get_surface().get_size()
        screen.fill(GRAY)

        """
        ZOOM_CONSTANT
        How zoomed in should the UI be.
        Value is maximum distance between the current coordinate position of the car and any point on the track
        Determines how far ahead/behind the car the path should be shown
        """

        ZOOM_CONSTANT = 0.001
        ROTATION_ANGLE = -90 + -rotation_from_north # by default the rotation angle is -90 if the car is pointing NORTH

        i = 0

        prev = current_position

        seen = set()

        if i == len(points)-1:
            i = 0
            ROTATION_ANGLE = self.calculate_true_bearing(prev, current_position)
            prev = current_position

        i += 1

        points = [(a, b) for a, b in points if not (b in seen or seen.add(b))] # remove one point if two points have the same y value

        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    print("ESC was pressed. quitting...")
                    quit()

            if event.type == pygame.MOUSEBUTTONDOWN:
                # BUTTONS
                mouse = pygame.mouse.get_pos()

                # Decrease Zoom
                if 20 <= mouse[0] <= 80 and HEIGHT-60 <= mouse[1] <= HEIGHT-20:
                    ZOOM_CONSTANT = ZOOM_CONSTANT * 0.8

                # Increase Zoom
                if 80 <= mouse[0] <= 140 and HEIGHT-60 <= mouse[1] <= HEIGHT-20:
                    ZOOM_CONSTANT = ZOOM_CONSTANT * 1.2

                # Decrease Rotation
                if 140 <= mouse[0] <= 200 and HEIGHT-60 <= mouse[1] <= HEIGHT-20:
                    ROTATION_ANGLE = ROTATION_ANGLE - 5

                # Increase Rotation
                if 200 <= mouse[0] <= 260 and HEIGHT-60 <= mouse[1] <= HEIGHT-20:
                    ROTATION_ANGLE = ROTATION_ANGLE + 5

        screen.fill(GRAY) # redraw screen
        valid_points = self.get_points_in_range(points, current_position, ZOOM_CONSTANT)
        car_position = self.scale_point(current_position, valid_points, ZOOM_CONSTANT)

        pygame.draw.circle(screen, RED, car_position, 30, 30)

        radar = (WIDTH/2,HEIGHT/2)
        radar_len = 50
        x = radar[0] + math.cos(math.radians(ROTATION_ANGLE)) * radar_len
        y = radar[1] + math.sin(math.radians(ROTATION_ANGLE)) * radar_len

        # then render the line radar->(x,y)
        pygame.draw.line(screen, BLACK, radar, (x,y), 8)
        pygame.draw.line(screen, RED, radar, (radar[0],radar[1]-60), 8)

     

        if len(valid_points) > 1:
            scaled = self.scale_points(valid_points, current_position, ZOOM_CONSTANT)
            rotated = self.rotate_points(scaled, -ROTATION_ANGLE-90, car_position)

            for point in rotated:
                pygame.draw.circle(screen, BLUE, point, 5, 5)

     

        # screen extras
        font = pygame.font.SysFont(None, 24)
        current_label = font.render('Current Position: ' + str(current_position), True, BLUE)
        speed_label = font.render('Current Speed: ' + str(100), True, BLUE)
        zoom_label = font.render('Zoom: ' + str(ZOOM_CONSTANT), True, RED)
        rotation_label = font.render('Rotation (degrees): ' + str(ROTATION_ANGLE), True, RED)
        zoom_change_label = font.render('Adjust Zoom', True, BLUE)

        pygame.draw.rect(screen, RED, [20,HEIGHT-60,60,40])
        pygame.draw.rect(screen, GREEN, [80,HEIGHT-60,60,40])

        rotation_change_label = font.render('Adjust Rotation', True, BLUE)

        pygame.draw.rect(screen, RED, [140,HEIGHT-60,60,40])
        pygame.draw.rect(screen, GREEN, [200,HEIGHT-60,60,40])

        screen.blit(current_label, (20, 20))
        screen.blit(speed_label, (20, 60))
        screen.blit(zoom_label, (20, 100))
        screen.blit(rotation_label, (20, 140))
        screen.blit(rotation_change_label, (140, HEIGHT-80))
        screen.blit(zoom_change_label, (20, HEIGHT-80))

        pygame.display.update()

        # pygame.time.wait(UPDATE_DELAY)

 

    def get_points_in_range(self, points, center, maxRange):
        # given a list of points, return all the points that are within range of a center point
        valid_points = []

        for point in points:
            distance = math.sqrt((point[0] - center[0])**2 + (point[1] - center[1])**2)
            if distance <= maxRange:
                valid_points.append(point)

        return valid_points

    def scale_point(self, point, points, ZOOM_CONSTANT):
        # scale one point from a set of points
        # IMPORTANT: point must be the center point

        WIDTH, HEIGHT = pygame.display.get_surface().get_size()
        pointsXLow, pointsYLow, pointsXHigh, pointsYHigh = point[0]-ZOOM_CONSTANT, point[1]-ZOOM_CONSTANT, point[0]+ZOOM_CONSTANT, point[1]+ZOOM_CONSTANT
        outputXLow, outputYLow, outputXHigh, outputYHigh = 0, 0, WIDTH, HEIGHT

        scaled_points = []
        x = ((point[0] - pointsXLow) * (outputXHigh - outputXLow)) / (pointsXHigh - pointsXLow) + outputXLow
        y = ((point[1] - pointsYLow) * (outputYHigh - outputYLow)) / (pointsYHigh - pointsYLow) + outputYLow

        return (int(x),int(y))

    def scale_points(self, points, center, ZOOM_CONSTANT):

        # scale a set of given coordinates to fit within the screen
        WIDTH, HEIGHT = pygame.display.get_surface().get_size()
        pointsXLow, pointsYLow, pointsXHigh, pointsYHigh = center[0]-ZOOM_CONSTANT, center[1]-ZOOM_CONSTANT, center[0]+ZOOM_CONSTANT, center[1]+ZOOM_CONSTANT
        outputXLow, outputYLow, outputXHigh, outputYHigh = 0, 0, WIDTH, HEIGHT
        scaled_points = []

        for point in points:
            x = ((point[0] - pointsXLow) * (outputXHigh - outputXLow)) / (pointsXHigh - pointsXLow) + outputXLow
            y = ((point[1] - pointsYLow) * (outputYHigh - outputYLow)) / (pointsYHigh - pointsYLow) + outputYLow
            scaled_points.append((x,y))

        return scaled_points

    def rotate_points(self, points, angle, about):

        """
        Rotate a set of points in a plane by a certain angle about an arbitrary point.
        """

        # Convert angle to radians
        angle = math.radians(angle)

        # Calculate the rotation matrix

        rotation_matrix = [
            [math.cos(angle), -math.sin(angle)],
            [math.sin(angle), math.cos(angle)]
        ]

        # Calculate the rotated points
        rotated_points = []

        for point in points:
            rotated_points.append(
                [
                    int(rotation_matrix[0][0] * (point[0] - about[0]) + rotation_matrix[0][1] * (point[1] - about[1]) + about[0]),
                    int(rotation_matrix[1][0] * (point[0] - about[0]) + rotation_matrix[1][1] * (point[1] - about[1]) + about[1])
                ]
            )

        return rotated_points

 

    def calculate_true_bearing(self, coord1, coord2):
        lat1 = math.radians(coord1[0])
        lat2 = math.radians(coord2[0])
        long1 = math.radians(coord1[1])
        long2 = math.radians(coord2[1])

        x = math.cos(lat2) * math.sin(long2 - long1)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(long2 - long1)

        bearing = math.atan2(x, y)
        bearing = math.degrees(bearing)
        bearing = (bearing + 360) % 360

        return bearing

 
def main(args=None):

    rclpy.init(args=args)

    driver_ui = DriverUI()
    rclpy.spin(driver_ui)

    driver_ui.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':

    main()
