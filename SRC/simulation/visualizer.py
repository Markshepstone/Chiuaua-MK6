import pygame
import numpy as np
from config import TrackConfig, CarState

class Visualizer:
    def __init__(self):
        pygame.init()
        self.window_size = 800
        self.scale = self.window_size / TrackConfig.TRACK_SIZE
        self.screen = pygame.display.set_mode((self.window_size, self.window_size))
        pygame.display.set_caption("WRO Car Simulation")
        self.show_positions = False
        self.show_grid = False
        self.show_measurements = True  # Default to showing measurements
        
    def to_screen_coords(self, x, y):
        # Direct mapping from track to screen coordinates
        screen_x = x * self.scale
        screen_y = y * self.scale
        return (int(screen_x), int(screen_y))
        
    def _draw_track(self):
        # Draw outer border
        pygame.draw.rect(self.screen, (0, 0, 0), 
                        (0, 0, self.window_size, self.window_size), 2)
        
        # Draw center no-go zone
        center_size = int(TrackConfig.CENTER_SIZE * self.scale)
        center_pos = (self.window_size - center_size) // 2
        pygame.draw.rect(self.screen, (0, 0, 0),
                        (center_pos, center_pos, center_size, center_size))
        
        # Draw grid points for blocks
        for x, y in TrackConfig.BLOCK_POSITIONS:
            screen_x, screen_y = self.to_screen_coords(x, y)
            pygame.draw.circle(self.screen, (200, 200, 200), (screen_x, screen_y), 5)
    
    def _draw_car(self, car_state):
        # Use trueAngle for visualization
        angle = np.radians(car_state.trueAngle)
        
        # Calculate rectangle points for car representation
        car_length = int(TrackConfig.CAR_LENGTH * self.scale)
        car_width = int(TrackConfig.CAR_WIDTH * self.scale)
        x, y = self.to_screen_coords(car_state.x, car_state.y)
        
        # Calculate corners of the rectangle
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        
        # Calculate corner points
        points = [
            (x - car_length/2 * cos_a - car_width/2 * sin_a,
             y - car_length/2 * sin_a + car_width/2 * cos_a),
            (x + car_length/2 * cos_a - car_width/2 * sin_a,
             y + car_length/2 * sin_a + car_width/2 * cos_a),
            (x + car_length/2 * cos_a + car_width/2 * sin_a,
             y + car_length/2 * sin_a - car_width/2 * cos_a),
            (x - car_length/2 * cos_a + car_width/2 * sin_a,
             y - car_length/2 * sin_a - car_width/2 * cos_a),
        ]
        
        # Draw car as rectangle
        pygame.draw.polygon(self.screen, (0, 255, 0), points)
        
        # Draw direction indicator (front of car)
        front_x = x + (car_length/2) * cos_a
        front_y = y + (car_length/2) * sin_a
        pygame.draw.circle(self.screen, (255, 0, 0), (int(front_x), int(front_y)), 5)
    
    def _draw_blocks(self, blocks):
        block_size = int(TrackConfig.BLOCK_SIZE * self.scale)
        
        # Track segments (0-3) and positions/lanes (0-2)
        for segment in range(4):
            for lane in range(3):
                block_value = blocks[segment][lane]
                if block_value == 0:
                    continue
                    
                # Map segment and lane to track coordinates
                if segment == 0:  # Left side
                    x = 0
                    y = TrackConfig.GRID_SIZE + (lane * TrackConfig.GRID_SIZE)
                elif segment == 1:  # Top side
                    x = TrackConfig.GRID_SIZE + (lane * TrackConfig.GRID_SIZE)
                    y = 0
                elif segment == 2:  # Right side
                    x = TrackConfig.TRACK_SIZE
                    y = TrackConfig.GRID_SIZE + (lane * TrackConfig.GRID_SIZE)
                else:  # Bottom side
                    x = TrackConfig.GRID_SIZE + (lane * TrackConfig.GRID_SIZE)
                    y = TrackConfig.TRACK_SIZE
                    
                # Convert block values to colors and draw
                color = (0, 255, 0) if block_value == 71 else \
                       (255, 0, 0) if block_value == 82 else \
                       (255, 0, 255)  # Magenta
                
                screen_x, screen_y = self.to_screen_coords(x, y)
                pygame.draw.rect(self.screen, color,
                               (screen_x - block_size//2, screen_y - block_size//2,
                                block_size, block_size))
    
    def draw_frame(self, car_state, blocks, lidar_readings, is_colliding=False):
        # Clear screen
        self.screen.fill((255, 255, 255))
        
        # Draw coordinate grid first
        if self.show_grid:
            # Draw grid lines
            self._draw_grid_lines()
            # Draw coordinate cross after track elements
            self._draw_coordinate_cross()
        
        # Draw track elements
        self._draw_track()
        self._draw_valid_positions()
        self._draw_blocks(blocks)
        
        # Draw car and remaining elements
        self._draw_car(car_state)
        
        # Draw measurement lines if enabled
        if self.show_measurements and lidar_readings:
            self._draw_lidar(car_state, lidar_readings)
        
        # Draw collision state
        self._draw_collision_state(car_state, is_colliding)
        
        # Draw sensor information
        self.draw_lidar_info(lidar_readings)
        
        # Draw gyro information
        self.draw_gyro_info(car_state)
        self.draw_motor_info(car_state)
        
        # Update display
        pygame.display.flip()
    
    def draw_gyro_info(self, car_state):
        # Draw angle information at top of screen (lines 299-306)
        font = pygame.font.Font(None, 36)
        
        # Display actual angle
        angle_text = f"Act: {int(car_state.trueAngle)}"
        text_surface = font.render(angle_text, True, (0, 0, 0))
        self.screen.blit(text_surface, (10, 10))
        
        # Display target angle
        target_text = f"Tgt: {int(car_state.targetAngle)}"
        text_surface = font.render(target_text, True, (0, 0, 0))
        self.screen.blit(text_surface, (10, 50))
        
        # Draw steering indicator
        self._draw_steering(car_state)
    
    def _draw_steering(self, car_state):
        # Calculate steering angle based on Arduino code (lines 336-358)
        angle_diff = car_state.trueAngle - car_state.targetAngle
        
        # Apply steering limits
        max_steering = 60
        if angle_diff > max_steering:
            angle_diff = max_steering
        elif angle_diff < -max_steering:
            angle_diff = -max_steering
        
        # Calculate steering position
        multiplier = 0.5
        steering_angle = angle_diff * multiplier + 92
        
        # Draw steering indicator at bottom
        center_x = self.window_size // 2
        y = self.window_size - 60
        width = 100
        height = 20
        
        # Draw background bar
        pygame.draw.rect(self.screen, (200, 200, 200),
                        (center_x - width//2, y, width, height))
        
        # Draw indicator
        indicator_pos = center_x + (steering_angle - 92) * (width/120)
        pygame.draw.rect(self.screen, (0, 0, 255),
                        (indicator_pos - 5, y, 10, height))
    
    def update_leds(self, led_strip):
        # Draw LED strip at bottom of screen
        led_size = 20
        spacing = 5
        start_x = (self.window_size - (12 * (led_size + spacing))) // 2
        y = self.window_size - 30
        
        for i, color in enumerate(led_strip):
            x = start_x + i * (led_size + spacing)
            pygame.draw.rect(self.screen, color, 
                            (x, y, led_size, led_size))
    
    def draw_motor_info(self, car_state):
        # Draw motor speed at bottom right
        font = pygame.font.Font(None, 36)
        
        # Display motor speed and direction
        direction = "FWD" if car_state.speed >= 0 else "REV"
        speed_text = f"Motor: {abs(car_state.speed)} {direction}"
        text_surface = font.render(speed_text, True, (0, 0, 0))
        self.screen.blit(text_surface, (self.window_size - 150, self.window_size - 40))
        
        # Draw binary LED indicators for absolute speed value
        self._draw_motor_leds(abs(car_state.speed))

    def _draw_motor_leds(self, speed):
        # Draw binary LED indicators for motor speed
        led_size = 15
        spacing = 5
        start_x = self.window_size - 200
        y = self.window_size - 80
        
        # Clear LED backgrounds
        for i in range(4):
            x = start_x + i * (led_size + spacing)
            pygame.draw.rect(self.screen, (200, 200, 200),
                            (x, y, led_size, led_size))
        
        # Light up LEDs based on speed (binary encoding)
        abs_speed = abs(speed)
        if abs_speed >= 8:
            pygame.draw.rect(self.screen, (255, 0, 0),
                            (start_x, y, led_size, led_size))
        if abs_speed >= 4:
            pygame.draw.rect(self.screen, (255, 0, 0),
                            (start_x + led_size + spacing, y, led_size, led_size))
        if abs_speed >= 2:
            pygame.draw.rect(self.screen, (255, 0, 0),
                            (start_x + 2 * (led_size + spacing), y, led_size, led_size))
        if abs_speed >= 1:
            pygame.draw.rect(self.screen, (255, 0, 0),
                            (start_x + 3 * (led_size + spacing), y, led_size, led_size))
    
    def _draw_lidar(self, car_state, lidar_readings):
        """Draw LIDAR sensor readings"""
        # Convert sensor readings to screen coordinates
        x, y = self.to_screen_coords(car_state.x, car_state.y)
        font = pygame.font.Font(None, 24)
        
        # Draw each LIDAR reading
        for sensor_addr, distance in lidar_readings.items():
            if distance <= 0:
                continue
            
            # Get sensor number and angle
            if sensor_addr == 0x61:
                sensor_num = "1"
                angle = car_state.trueAngle
            elif sensor_addr == 0x62:
                sensor_num = "2"
                angle = car_state.trueAngle - 90
            elif sensor_addr == 0x60:
                sensor_num = "3"
                angle = car_state.trueAngle + 90
            elif sensor_addr == 0x63:
                sensor_num = "4"
                angle = car_state.trueAngle + 45
            elif sensor_addr == 0x59:
                sensor_num = "5"
                angle = car_state.trueAngle - 45
            elif sensor_addr == 0x65:
                sensor_num = "6"
                angle = car_state.trueAngle + 180
            
            # Calculate endpoint
            angle_rad = np.radians(angle)
            end_x = x + distance * self.scale * np.cos(angle_rad)
            end_y = y + distance * self.scale * np.sin(angle_rad)
            
            # Draw LIDAR beam
            pygame.draw.line(self.screen, (255, 0, 0), (x, y), (end_x, end_y), 2)
            
            # Draw sensor number at midpoint of beam
            mid_x = (x + end_x) / 2
            mid_y = (y + end_y) / 2
            num_surface = font.render(sensor_num, True, (255, 0, 0))
            self.screen.blit(num_surface, (mid_x, mid_y))
    
    def _draw_collision_state(self, car_state, is_colliding):
        if not is_colliding:
            return
        
        # Get car corners for collision outline
        angle = np.radians(car_state.trueAngle)
        car_length = int(TrackConfig.CAR_LENGTH * self.scale)
        car_width = int(TrackConfig.CAR_WIDTH * self.scale)
        x, y = self.to_screen_coords(car_state.x, car_state.y)
        
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        
        # Calculate corner points (same as _draw_car)
        points = [
            (x - car_length/2 * cos_a - car_width/2 * sin_a,
             y - car_length/2 * sin_a + car_width/2 * cos_a),
            (x + car_length/2 * cos_a - car_width/2 * sin_a,
             y + car_length/2 * sin_a + car_width/2 * cos_a),
            (x + car_length/2 * cos_a + car_width/2 * sin_a,
             y + car_length/2 * sin_a - car_width/2 * cos_a),
            (x - car_length/2 * cos_a + car_width/2 * sin_a,
             y - car_length/2 * sin_a - car_width/2 * cos_a),
        ]
        
        # Draw red outline
        pygame.draw.polygon(self.screen, (255, 0, 0), points, 3)
    
    def _draw_valid_positions(self):
        if not self.show_positions:
            return
            
        position_font = pygame.font.Font(None, 24)
        coord_font = pygame.font.SysFont('Arial', 16)  # Thinner font for coordinates
        
        for i, (x, y) in enumerate(TrackConfig.BLOCK_POSITIONS):
            screen_x, screen_y = self.to_screen_coords(x, y)
            # Draw larger circle
            pygame.draw.circle(self.screen, (255, 0, 0), (screen_x, screen_y), 8)
            # Draw position number
            text = position_font.render(str(i), True, (0, 0, 0))
            self.screen.blit(text, (screen_x + 10, screen_y - 10))
            # Draw coordinates in dark blue
            coord_text = coord_font.render(f"({x},{y})", True, (0, 0, 139))  # Dark blue RGB
            self.screen.blit(coord_text, (screen_x - 20, screen_y + 10))
    
    def toggle_positions(self):
        self.show_positions = not self.show_positions
    #Grid overlay helper when "o" is pressed
    def _draw_coordinate_grid(self):
        if not self.show_grid:
            return
        self._draw_grid_lines()
        self._draw_coordinate_cross()
    
    def draw_lidar_info(self, lidar_readings):
        if not lidar_readings:
            return
        
        # Use smaller font (20 instead of 28)
        font = pygame.font.Font(None, 20)
        
        # Calculate center block position and size
        center_size = int(TrackConfig.CENTER_SIZE * self.scale)
        center_pos = (self.window_size - center_size) // 2
        
        # Start position within center block (30% from left edge)
        inset_x = center_size * 0.3
        inset_y = center_size * 0.2
        x_start = center_pos + inset_x
        y_start = center_pos + inset_y
        
        # Display each LIDAR reading in white with sensor number
        for sensor_addr, distance in lidar_readings.items():
            if sensor_addr == 0x61:
                text = f"TF Luna #1 Front: {int(distance)}mm"
            elif sensor_addr == 0x62:
                text = f"TF Luna #2 Left: {int(distance)}mm"
            elif sensor_addr == 0x60:
                text = f"TF Luna #3 Right: {int(distance)}mm"
            elif sensor_addr == 0x63:
                text = f"TF Luna #4 Right45: {int(distance)}mm"
            elif sensor_addr == 0x59:
                text = f"TF Luna #5 Left45: {int(distance)}mm"
            elif sensor_addr == 0x65:
                text = f"TF Luna #6 Back: {int(distance)}mm"
                
            text_surface = font.render(text, True, (255, 255, 255))
            self.screen.blit(text_surface, (x_start, y_start))
            y_start += 25  # Reduced vertical spacing
    
    def _draw_grid_lines(self):
        font = pygame.font.Font(None, 24)
        grid_spacing = 500 * self.scale
        
        # Draw vertical lines and x-coordinates
        for x in range(0, TrackConfig.TRACK_SIZE + 1, 500):
            screen_x = int(x * self.scale)
            pygame.draw.line(self.screen, (200, 200, 200), 
                            (screen_x, 0), (screen_x, self.window_size), 1)
            text = font.render(f"{x}", True, (100, 100, 100))
            self.screen.blit(text, (screen_x + 5, 10))
        
        # Draw horizontal lines and y-coordinates
        for y in range(0, TrackConfig.TRACK_SIZE + 1, 500):
            screen_y = int(y * self.scale)
            pygame.draw.line(self.screen, (200, 200, 200), 
                            (0, screen_y), (self.window_size, screen_y), 1)
            text = font.render(f"{y}", True, (100, 100, 100))
            self.screen.blit(text, (10, screen_y + 5))
    
    def _draw_coordinate_cross(self):
        # Draw direction cross on right side
        cross_size = 50
        angle_font = pygame.font.SysFont('Arial', 14)
        
        # Position cross on right side
        center_x = self.window_size - 100  # 100 pixels from right edge
        center_y = 100  # 100 pixels from top
        
        # Draw cross lines
        pygame.draw.line(self.screen, (100, 100, 255), 
                        (center_x, center_y - cross_size), 
                        (center_x, center_y + cross_size), 2)  # Vertical
        pygame.draw.line(self.screen, (100, 100, 255), 
                        (center_x - cross_size, center_y), 
                        (center_x + cross_size, center_y), 2)  # Horizontal
        
        # Add angle labels
        angles = {
            (0, -cross_size - 15): "0°",
            (cross_size + 5, 0): "90°",
            (0, cross_size + 15): "180°",
            (-cross_size - 25, 0): "270°"
        }
        
        for (dx, dy), label in angles.items():
            text = angle_font.render(label, True, (100, 100, 255))
            self.screen.blit(text, (center_x + dx, center_y + dy))
    
