"""
Angle Converter Module - Converts steering angles to ESP32 servo values.

Following SRP: This module only handles angle conversion.
"""

# ESP32 Servo constants
# NOTE: 50 = right turn, 160 = left turn (inverted from typical convention)
SERVO_CENTER = 105
SERVO_RIGHT = 50   # Right turn (lower value)
SERVO_LEFT = 160   # Left turn (higher value)

# Steering angle constants
STRAIGHT_THRESHOLD = 6.0  # If angle is within ±6 degrees, go straight
ANGLE_MIN = -30  # Minimum steering angle (degrees)
ANGLE_MAX = 30   # Maximum steering angle (degrees)


class AngleConverter:
    """Converts steering angles to ESP32 servo angles."""
    
    def __init__(self):
        """Initialize the angle converter."""
        # Calculate conversion factor: servo range / angle range
        # Servo range: 160 - 50 = 110 degrees (55 each side from center)
        # Angle range: 30 - (-30) = 60 degrees (30 each side)
        # Conversion: 55 / 30 ≈ 1.83 degrees servo per degree steering angle
        # NOTE: Since 50 is right and 160 is left, we need to invert the conversion
        # Positive angle (right turn) → lower servo value (50)
        # Negative angle (left turn) → higher servo value (160)
        self.conversion_factor = (SERVO_CENTER - SERVO_RIGHT) / ANGLE_MAX
    
    def convert(self, steering_angle: float) -> int:
        """
        Convert steering angle to ESP32 servo angle.
        
        Args:
            steering_angle: Steering angle from curvature (-30 to +30 degrees, or 0 for straight)
            
        Returns:
            Servo angle (50-160, where 105 is center)
        """
        # Special case: angle close to 0 means "go straight" → servo center
        if abs(steering_angle) <= STRAIGHT_THRESHOLD:
            return SERVO_CENTER
        
        # Convert steering angle to servo angle
        # NOTE: Servo is inverted: 50 = right, 160 = left
        # Positive angle (right turn) → subtract from center → lower value (50)
        # Negative angle (left turn) → add to center → higher value (160)
        # Formula: servo_angle = center - (steering_angle * conversion_factor)
        servo_angle = SERVO_CENTER - (steering_angle * self.conversion_factor)
        
        # Clamp to valid range (SERVO_RIGHT=50 is minimum, SERVO_LEFT=160 is maximum)
        servo_angle = max(SERVO_RIGHT, min(SERVO_LEFT, int(servo_angle)))
        
        return servo_angle
    
    def get_conversion_info(self) -> dict:
        """Get conversion information for debugging."""
        return {
            'servo_center': SERVO_CENTER,
            'servo_left': SERVO_LEFT,
            'servo_right': SERVO_RIGHT,
            'straight_threshold': STRAIGHT_THRESHOLD,
            'angle_min': ANGLE_MIN,
            'angle_max': ANGLE_MAX,
            'conversion_factor': self.conversion_factor
        }

