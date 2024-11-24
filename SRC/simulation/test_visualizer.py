import pygame
import time
from visualizer import Visualizer
from config import TrackConfig, CarState

class TestCarState:
    def __init__(self):
        self.x = 1500  # Center horizontally
        self.y = 3000  # Start at bottom
        self.heading = 0
        self.lane = 1

def main():
    vis = Visualizer()
    car = TestCarState()
    
    # Sample blocks
    blocks = [
        ((-1000, -750), 'red'),
        ((1000, 750), 'green'),
    ]
    
    # Sample LIDAR readings
    lidar_readings = {
        0: 500,    # front
        90: 300,   # right
        -90: 300,  # left
    }
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # Update car position for testing
        car.heading += 1
        
        vis.draw_frame(car, blocks, lidar_readings)
        time.sleep(0.016)  # ~60 FPS
    
    pygame.quit()

if __name__ == "__main__":
    main()
