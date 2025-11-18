"""
Angle Converter Module - Converts PID angles to ESP32 servo values.

Following SRP: This module only handles angle conversion.
"""

# ESP32 Servo constants
# NOTE: 50 = right turn, 135 = left turn (inverted from typical convention)
SERVO_CENTER = 105
SERVO_RIGHT = 50   # Right turn (lower value)
SERVO_LEFT = 135   # Left turn (higher value)

# PID angle constants
PID_STRAIGHT = -3  # Special value for "go straight"
PID_MIN = -22
PID_MAX = 22


class AngleConverter:
    """Converts PID steering angles to ESP32 servo angles."""
    
    def __init__(self):
        """Initialize the angle converter."""
        # Calculate conversion factor: servo range / PID range
        # Servo range: 135 - 50 = 85 degrees (42.5 each side from center)
        # PID range: 22 - (-22) = 44 degrees (22 each side)
        # Conversion: 42.5 / 22 ≈ 1.93 degrees servo per degree PID
        # NOTE: Since 50 is right and 135 is left, we need to invert the conversion
        # Positive PID angle (right turn) → lower servo value (50)
        # Negative PID angle (left turn) → higher servo value (135)
        self.conversion_factor = (SERVO_CENTER - SERVO_RIGHT) / PID_MAX
    
    def convert(self, pid_angle: float) -> int:
        """
        Convert PID angle to ESP32 servo angle.
        
        Args:
            pid_angle: PID controller angle (-22 to +22, or -3 for straight)
            
        Returns:
            Servo angle (50-135, where 105 is center)
        """
        # Special case: PID angle -3 means "go straight" → servo center
        if abs(pid_angle - PID_STRAIGHT) < 0.5:
            return SERVO_CENTER
        
        # Convert PID angle to servo angle
        # NOTE: Servo is inverted: 50 = right, 135 = left
        # Positive PID angle (right turn) → subtract from center → lower value (50)
        # Negative PID angle (left turn) → add to center → higher value (135)
        # Formula: servo_angle = center - (pid_angle * conversion_factor)
        servo_angle = SERVO_CENTER - (pid_angle * self.conversion_factor)
        
        # Clamp to valid range (SERVO_RIGHT=50 is minimum, SERVO_LEFT=135 is maximum)
        servo_angle = max(SERVO_RIGHT, min(SERVO_LEFT, int(servo_angle)))
        
        return servo_angle
    
    def get_conversion_info(self) -> dict:
        """Get conversion information for debugging."""
        return {
            'servo_center': SERVO_CENTER,
            'servo_left': SERVO_LEFT,
            'servo_right': SERVO_RIGHT,
            'pid_straight': PID_STRAIGHT,
            'pid_min': PID_MIN,
            'pid_max': PID_MAX,
            'conversion_factor': self.conversion_factor
        }

