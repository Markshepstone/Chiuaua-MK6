import pygame
from wro_simulation import WROSimulation

def main():
    sim = WROSimulation()
    
    # Initialize from Arduino setup (lines 72-121)
    sim.car_state.lane = 3
    sim.car_state.currentLane = 3
    sim.car_state.targetAngle = 0
    sim.car_state.speed = 0
    
    running = True
    clock = pygame.time.Clock()
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                # Lane controls
                if event.key == pygame.K_1:
                    sim.car_state.lane = 0  # Left lane
                elif event.key == pygame.K_2:
                    sim.car_state.lane = 3  # Middle lane
                elif event.key == pygame.K_3:
                    sim.car_state.lane = 1  # Right lane
                
                # Motor controls
                elif event.key == pygame.K_UP:
                    sim.car_state.speed = 7  # Full speed forward
                elif event.key == pygame.K_DOWN:
                    sim.car_state.speed = 0  # Stop
                elif event.key == pygame.K_SPACE:
                    sim.car_state.speed = -1  # Reverse
                
                # Turning controls
                elif event.key == pygame.K_LEFT:
                    sim.car_state.targetAngle -= 90
                elif event.key == pygame.K_RIGHT:
                    sim.car_state.targetAngle += 90
                
                # Special functions
                elif event.key == pygame.K_b:
                    sim.updateLaneMidBlock(sim.currentSection)  # Block detection (lines 795-830)
                
        # Update simulation state
        sim.update()
        sim.updateLEDLane()  # Update LED visualization (lines 833-847)
        
        # Draw current state
        sim.visualizer.draw_frame(
            sim.car_state,
            sim.blocks,
            sim.get_lidar_readings()
        )
        
        clock.tick(60)
    
    pygame.quit()

if __name__ == "__main__":
    main()
