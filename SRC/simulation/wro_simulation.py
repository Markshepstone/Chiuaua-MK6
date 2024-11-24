from pathlib import Path
import pygame
import numpy as np
from visualizer import Visualizer
from config import TrackConfig, CarState
import random
import time

class WROSimulation:
    def __init__(self, arduino_code_path="Obsticle Challenge Code"):
        self.code_path = Path(arduino_code_path)
        self.visualizer = Visualizer()
        self.car_state = CarState()
        self.led_strip = [(0, 0, 0)] * 12  # Simulating 12 LEDs as in Arduino (line 25)
        self.initialize_from_arduino()
        self.generate_random_blocks()
        self.last_lidar_update = 0
        self.lidar_readings = self.get_lidar_readings()  # Initial readings
        
    def initialize_from_arduino(self):
        # Initialize values from Arduino code
        self.car_state.lane = 3        # From line 133
        self.car_state.currentLane = 3 # From line 134
        self.car_state.targetAngle = 0 # From line 135
        self.blocks = [[0] * 4 for _ in range(4)]  # From line 32
        
    def generate_random_blocks(self):
        """Generate block configuration from TrackConfig"""
        # Clear existing blocks
        self.blocks = [[0] * 4 for _ in range(4)]
        
        # Place blocks according to configuration
        for segment, (block_type, lane) in TrackConfig.BLOCK_LAYOUT.items():
            self.blocks[segment][lane] = block_type
        
    def updateLEDLane(self):
        # Clear lane indicator LEDs (lines 835-837)
        for i in range(9, 12):
            self.led_strip[i] = (0, 0, 0)
        
        # Update LED based on lane (lines 839-845)
        if self.car_state.lane == 0:
            self.led_strip[9] = (255, 255, 255)  # Left lane
        elif self.car_state.lane == 1:
            self.led_strip[11] = (255, 255, 255)  # Right lane
        else:
            self.led_strip[10] = (255, 255, 255)  # Middle lane
            
        # In simulation, we'll pass LED states to visualizer
        self.visualizer.update_leds(self.led_strip)
        
    def update(self):
        # Update gyro first
        self.update_gyro()
        
        # Store previous position for collision check
        prev_x = self.car_state.x
        prev_y = self.car_state.y
        
        # Calculate time delta since last update (for 60 FPS)
        current_time = time.time()
        dt = current_time - self.last_update_time if hasattr(self, 'last_update_time') else 1/60
        self.last_update_time = current_time
        
        # Then update car position based on true angle
        if hasattr(self.car_state, 'speed'):
            angle_rad = np.radians(self.car_state.trueAngle)
            # Speed is in mm/s, multiply by time delta for frame-independent movement
            speed_factor = self.car_state.speed * TrackConfig.SPEED * dt
            self.car_state.x += np.cos(angle_rad) * speed_factor
            self.car_state.y += np.sin(angle_rad) * speed_factor
        
        # Check for collision
        is_colliding = self.check_collision()
        if is_colliding:
            self.car_state.x = prev_x
            self.car_state.y = prev_y
        
        # Update LIDAR readings every 500ms
        current_time = pygame.time.get_ticks()
        if current_time - self.last_lidar_update >= 500:
            self.lidar_readings = self.get_lidar_readings()
            self.last_lidar_update = current_time
        
        # Draw frame
        self.visualizer.draw_frame(
            self.car_state,
            self.blocks,
            self.lidar_readings,
            is_colliding
        )
        
    def get_lidar_readings(self):
        readings = {}
        for sensor_addr, angle_offset in [
            (0x61, 0),     #1 front
            (0x62, -90),   #2 left
            (0x60, 90),    #3 right
            (0x63, 45),    #4 right45
            (0x59, -45),   #5 left45
            (0x65, 180),   #6 back
        ]:
            # Calculate actual angle including car rotation
            actual_angle = self.car_state.trueAngle + angle_offset
            
            # Get raw distance (before wall collision)
            raw_distance = 8000  # TF-Luna max range is 8m = 8000mm
            
            # Check for wall/center collisions
            actual_distance = self.check_beam_collision(
                self.car_state.x, 
                self.car_state.y,
                actual_angle,
                raw_distance
            )
            
            readings[sensor_addr] = actual_distance
        
        return readings
    
    def update_gyro(self):
        # Simulate BNO08x gyro behavior
        angle_diff = self.car_state.targetAngle - self.car_state.trueAngle
        
        # Calculate minimum turning radius based on car dimensions
        min_radius = TrackConfig.CAR_LENGTH / np.tan(np.radians(30))  # 30° max steering
        max_steering = np.degrees(np.arctan(TrackConfig.CAR_LENGTH / min_radius))
        
        if angle_diff > max_steering:
            angle_diff = max_steering
        elif angle_diff < -max_steering:
            angle_diff = -max_steering
        
        # Steering multiplier (line 347)
        multiplier = 0.5
        
        # Calculate steering angle (lines 352-358)
        steering_angle = angle_diff * multiplier + 92
        
        # Update true angle based on steering
        if abs(angle_diff) > 1:
            # Gradually approach target angle
            self.car_state.trueAngle += np.sign(angle_diff) * 2
        
        # Handle angle wrapping (lines 311-320)
        if self.car_state.trueAngle > 180:
            self.car_state.trueAngle -= 360
        elif self.car_state.trueAngle < -180:
            self.car_state.trueAngle += 360
        
    def check_collision(self):
        # Get car corners based on current position and angle
        angle_rad = np.radians(self.car_state.trueAngle)
        cos_a = np.cos(angle_rad)
        sin_a = np.sin(angle_rad)
        
        # Calculate car corners (dimensions from center)
        half_length = TrackConfig.CAR_LENGTH / 2
        half_width = TrackConfig.CAR_WIDTH / 2
        
        corners = [
            (self.car_state.x - half_length * cos_a - half_width * sin_a,
             self.car_state.y - half_length * sin_a + half_width * cos_a),
            (self.car_state.x + half_length * cos_a - half_width * sin_a,
             self.car_state.y + half_length * sin_a + half_width * cos_a),
            (self.car_state.x + half_length * cos_a + half_width * sin_a,
             self.car_state.y + half_length * sin_a - half_width * cos_a),
            (self.car_state.x - half_length * cos_a + half_width * sin_a,
             self.car_state.y - half_length * sin_a - half_width * cos_a)
        ]
        
        # Check wall collisions (0 to TRACK_SIZE)
        for x, y in corners:
            if x < 0 or x > TrackConfig.TRACK_SIZE or y < 0 or y > TrackConfig.TRACK_SIZE:
                return True
                
        # Check center zone collision
        center_start = (TrackConfig.TRACK_SIZE - TrackConfig.CENTER_SIZE) / 2
        center_end = center_start + TrackConfig.CENTER_SIZE
        
        for x, y in corners:
            if (center_start < x < center_end and 
                center_start < y < center_end):
                return True
        
        return False
    
    def check_beam_collision(self, start_x, start_y, angle, distance):
        angle_rad = np.radians(angle)
        
        # Calculate wall distances first
        wall_dist = self._check_wall_collision(start_x, start_y, angle_rad)
        
        # Check center block collision
        center_start = (TrackConfig.TRACK_SIZE - TrackConfig.CENTER_SIZE) / 2
        center_end = center_start + TrackConfig.CENTER_SIZE
        
        # Calculate beam endpoint
        end_x = start_x + distance * np.cos(angle_rad)
        end_y = start_y + distance * np.sin(angle_rad)
        
        # Check if beam intersects with center block
        if (self._line_intersects_box(
                start_x, start_y, end_x, end_y,
                center_start, center_start, center_end, center_end)):
            # Calculate distance to intersection point
            center_dist = self._distance_to_box(
                start_x, start_y, angle_rad,
                center_start, center_start, center_end, center_end)
            return min(wall_dist, center_dist)
        
        return wall_dist

    def _line_intersects_box(self, x1, y1, x2, y2, box_x1, box_y1, box_x2, box_y2):
        # Check if line segment intersects with box
        if (max(x1, x2) < box_x1 or min(x1, x2) > box_x2 or
            max(y1, y2) < box_y1 or min(y1, y2) > box_y2):
            return False
            
        # Check each edge of the box
        edges = [
            ((box_x1, box_y1), (box_x2, box_y1)),  # Top
            ((box_x2, box_y1), (box_x2, box_y2)),  # Right
            ((box_x2, box_y2), (box_x1, box_y2)),  # Bottom
            ((box_x1, box_y2), (box_x1, box_y1))   # Left
        ]
        
        for (ex1, ey1), (ex2, ey2) in edges:
            if self._line_segments_intersect(x1, y1, x2, y2, ex1, ey1, ex2, ey2):
                return True
        return False

    def _distance_to_box(self, x, y, angle_rad, box_x1, box_y1, box_x2, box_y2):
        # Calculate distances to all edges and return minimum
        distances = []
        cos_a = np.cos(angle_rad)
        sin_a = np.sin(angle_rad)
        
        # Check horizontal edges
        if sin_a != 0:
            t1 = (box_y1 - y) / sin_a
            t2 = (box_y2 - y) / sin_a
            if t1 > 0: distances.append(t1)
            if t2 > 0: distances.append(t2)
        
        # Check vertical edges
        if cos_a != 0:
            t3 = (box_x1 - x) / cos_a
            t4 = (box_x2 - x) / cos_a
            if t3 > 0: distances.append(t3)
            if t4 > 0: distances.append(t4)
        
        return min(distances) if distances else float('inf')
    
    def line_intersects_point(self, x0, y0, angle, px, py):
        """Helper to check if a line from (x0,y0) at angle intersects point (px,py)"""
        # Vector from start to point
        dx = px - x0
        dy = py - y0
        
        # Angle between vectors
        point_angle = np.arctan2(dy, dx)
        
        # Check if angles match (allowing for small floating point differences)
        return abs(angle - point_angle) < 0.01
    
    def _check_wall_collision(self, start_x, start_y, angle_rad):
        """Calculate distance to wall collision"""
        cos_a = np.cos(angle_rad)
        sin_a = np.sin(angle_rad)
        
        # Initialize wall distance as infinity
        wall_dist = float('inf')
        
        # Check horizontal walls
        if sin_a != 0:
            # Distance to top wall
            if sin_a > 0:
                ty = (TrackConfig.TRACK_SIZE - start_y) / sin_a
                if ty > 0:
                    wall_dist = min(wall_dist, ty)
            # Distance to bottom wall
            else:
                ty = -start_y / sin_a
                if ty > 0:
                    wall_dist = min(wall_dist, ty)
        
        # Check vertical walls
        if cos_a != 0:
            # Distance to right wall
            if cos_a > 0:
                tx = (TrackConfig.TRACK_SIZE - start_x) / cos_a
                if tx > 0:
                    wall_dist = min(wall_dist, tx)
            # Distance to left wall
            else:
                tx = -start_x / cos_a
                if tx > 0:
                    wall_dist = min(wall_dist, tx)
        
        return wall_dist
    
    def _line_segments_intersect(self, x1, y1, x2, y2, x3, y3, x4, y4):
        # Calculate denominators for intersection check
        denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
        if denom == 0:  # Lines are parallel
            return False
        
        ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
        ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom
        
        # Check if intersection point lies within both line segments
        return (0 <= ua <= 1) and (0 <= ub <= 1)