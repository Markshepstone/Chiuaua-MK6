class TrackConfig:
    # Track dimensions in mm
    TRACK_SIZE = 3000
    CENTER_SIZE = 1000
    GRID_SIZE = 500    # Distance between block positions
    
    # Car specifications
    CAR_LENGTH = 210   # Updated car length
    CAR_WIDTH = 200    # Updated car width
    TURNING_RADIUS = 300
    SPEED = 20     # speed travelled in mm per second
    
    # Block specifications
    BLOCK_SIZE = 50
    MAX_BLOCKS = 7
    
    # Valid block positions (x, y) in mm from top-left
    BLOCK_POSITIONS = [
        # (segment 0)
        (1000, 400),    # Lane 0 position 1 (left)
        (1500, 400),   # Lane 0 position 2 (left)
        (2000, 400),   # Lane 0 position 3 (left)
        (1000, 600),   # Lane 1 position 4 (right)
        (1500, 600),   # Lane 1 position 5 (right)
        (2000, 600),   # Lane 1 position 6 (right)
        
        # (segment 1)
        (2600, 1000),  # Lane 0 position 7 (left)
        (2600, 1500), # Lane 0 position 8 (left)
        (2600, 2000), # Lane 0 position 9 (left)
        (2400, 1000), # Lane 1 position 10 (right)
        (2400, 1500), # Lane 1 position 11 (right)
        (2400, 2000), # Lane 1 position 12 (right)
        
        # (segment 2)
        (2000, 2600),  # Lane 0 position 13 (left)
        (1500, 2600), # Lane 0 position 14 (left)
        (1000, 2600), # Lane 0 position 15 (left)
        (2000, 2400), # Lane 1 position 16 (right)
        (1500, 2400), # Lane 1 position 17 (right)
        (1000, 2400), # Lane 1 position 18 (right)
        
        # (segment 3)
        (400, 2000),  # Lane 0 position 19 (left)
        (400, 1500), # Lane 0 position 20 (left)
        (400, 1000), # Lane 0 position 21 (left)
        (600, 2000), # Lane 1 position 22 (right)
        (600, 1500), # Lane 1 position 23 (right)
        (600, 1000), # Lane 1 position 24 (right)
    ]
    
    # Block configuration
    BLOCK_LAYOUT = {
        # Format: segment: (block_type, lane)
        # block_type: 82=RED, 71=GREEN, 77=MAGENTA
        # lane: 0=left, 1=middle, 2=right
        0: (82, 1),  # Red block in bottom-left segment, middle lane
        1: (71, 0),  # Green block in top-left segment, left lane
        2: (82, 2),  # Red block in top-right segment, right lane
        3: (77, 1),  # Magenta block in bottom-right segment, middle lane
    }

class CarState:
    def __init__(self):
        self.x = 1200      # Center horizontally
        self.y = 800      # Start at bottom
        self.speed = 0
        self.lane = 3
        self.currentLane = 3
        
        # Get start conditions
        start_conditions = randomised_startconditions()
        
        # Set initial angle based on startDirection
        # Note: Gyro will show 0, but car's visual direction will match startDirection
        if start_conditions.startDirection == "clockwise":
            self.x = 1500          # Center horizontally
            self.targetAngle = 0   # Facing right
            self.trueAngle = 0     # Actual car direction
        else:
            self.x = 1500          # Center horizontally
            self.targetAngle = 0   # Gyro shows 0
            self.trueAngle = 180   # But car faces left
            
        self.heading = 0


class randomised_startconditions:
    def __init__(self):
        self.startDirection = "clockwise"
    