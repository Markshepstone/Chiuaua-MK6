import pygame
from wro_simulation import WROSimulation
import time

def main():
    # Initialize simulation
    sim = WROSimulation()
    
    # Main game loop
    running = True
    auto_mode = False
    
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                # Reset command (always available)
                if event.key == pygame.K_r:
                    sim = WROSimulation()
                    auto_mode = False
                
                # Start/stop autonomous mode
                elif event.key == pygame.K_s:
                    auto_mode = not auto_mode  # Toggle auto_mode
                    sim.car_state.speed = 7 if auto_mode else 0  # Set speed based on mode
                
                # Manual controls only when not in auto mode
                elif not auto_mode:
                    if event.key == pygame.K_UP:
                        sim.car_state.speed = 7
                    elif event.key == pygame.K_DOWN:
                        sim.car_state.speed = 0
                    elif event.key == pygame.K_SPACE:
                        sim.car_state.speed = -1
                    elif event.key == pygame.K_LEFT:
                        sim.car_state.targetAngle -= 90
                    elif event.key == pygame.K_RIGHT:
                        sim.car_state.targetAngle += 90
                    elif event.key == pygame.K_1:
                        sim.car_state.lane = 0
                    elif event.key == pygame.K_2:
                        sim.car_state.lane = 1
                    elif event.key == pygame.K_3:
                        sim.car_state.lane = 3
                
                # Visualization toggles (always available)
                if event.key == pygame.K_p:
                    sim.visualizer.toggle_positions()
                elif event.key == pygame.K_o:
                    sim.visualizer.show_grid = not sim.visualizer.show_grid
                elif event.key == pygame.K_m:
                    sim.visualizer.show_measurements = not sim.visualizer.show_measurements
        
        # Update simulation
        if auto_mode:
            # Autonomous driving logic
            front_dist = sim.lidar_readings.get(0x61, float('inf'))
            left_dist = sim.lidar_readings.get(0x62, float('inf'))
            right_dist = sim.lidar_readings.get(0x60, float('inf'))
            
            if front_dist < 175:
                if sim.car_state.lane == 0:  # Left lane
                    sim.car_state.targetAngle -= 90
                elif sim.car_state.lane == 1:  # Right lane
                    sim.car_state.targetAngle += 90
        
        sim.update()
        
        # Cap framerate
        time.sleep(0.016)  # ~60 FPS

    pygame.quit()

if __name__ == "__main__":
    main()
