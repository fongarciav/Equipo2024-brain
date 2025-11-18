"""
Angle Converter Module - Converts PID angles to ESP32 servo values.

Following SRP: This module only handles angle conversion.
"""

# ESP32 Servo constants
# NOTE: 50 = right turn, 160 = left turn (inverted from typical convention)
SERVO_CENTER = 105
SERVO_RIGHT = 50   # Right turn (lower value)
SERVO_LEFT = 160   # Left turn (higher value)

# PID angle constants
PID_STRAIGHT = -3  # Special value for "go straight"
PID_MIN = -30  # Increased from -22 for more aggressive turns
PID_MAX = 30   # Increased from 22 for more aggressive turns


class AngleConverter:
    """Converts PID steering angles to ESP32 servo angles."""
    
    def __init__(self):
        """Initialize the angle converter."""
        # Calculate conversion factor: servo range / PID range
        # Servo range: 160 - 50 = 110 degrees (55 each side from center)
        # PID range: 30 - (-30) = 60 degrees (30 each side)
        # Use more of the servo range for aggressive turns: ~1.42 degrees servo per degree PID
        # NOTE: Since 50 is right and 160 is left, we need to invert the conversion
        # Positive PID angle (right turn) → lower servo value (50)
        # Negative PID angle (left turn) → higher servo value (160)
        # Using aggressive factor to utilize more servo range
        self.conversion_factor = (SERVO_CENTER - SERVO_RIGHT) / PID_MAX
    
    def convert(self, pid_angle: float) -> int:
        """
        Convert steering angle to ESP32 servo angle.
        
        Args:
            pid_angle: Steering angle from curvature (-30 to +30 degrees, or 0 for straight)
            
        Returns:
            Servo angle (50-160, where 105 is center)
        """
        # Special case: angle between -6 and 6 degrees means "go straight" → servo center
        if abs(pid_angle) <= 6.0:
            return SERVO_CENTER
        
        # Convert PID angle to servo angle
        # NOTE: Servo is inverted: 50 = right, 160 = left
        # Positive PID angle (right turn) → subtract from center → lower value (50)
        # Negative PID angle (left turn) → add to center → higher value (160)
        # Formula: servo_angle = center - (pid_angle * conversion_factor)
        servo_angle = SERVO_CENTER - (pid_angle * self.conversion_factor)
        
        # Clamp to valid range (SERVO_RIGHT=50 is minimum, SERVO_LEFT=160 is maximum)
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

