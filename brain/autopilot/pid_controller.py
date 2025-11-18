"""
PID Controller Module - Implements PID control for lane following.

Following SRP: This module only handles PID control logic.
"""


class PIDController:
    """
    Improved PID Controller with better stability and noise filtering.
    Features:
    - Anti-windup protection for integral term
    - Derivative filtering to reduce noise sensitivity
    - Dead zone handling
    - Output clamping
    """
    def __init__(self, Kp: float, Ki: float, Kd: float, tolerance: float, 
                 max_output: float = 30.0, min_output: float = -30.0):
        """
        Initialize the PID controller.
        
        Args:
            Kp: Proportional gain
            Ki: Integral gain
            Kd: Derivative gain
            tolerance: Dead zone tolerance (if error < tolerance, output is 0)
            max_output: Maximum output value (default: 30.0)
            min_output: Minimum output value (default: -30.0)
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.tolerance = tolerance
        
        # Output limits
        self.max_output = max_output
        self.min_output = min_output
        
        # State variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_derivative = 0.0
        
        # Derivative filter (low-pass filter to reduce noise)
        self.derivative_alpha = 0.7  # Filter coefficient (0-1, higher = less filtering)
        
        # Anti-windup: limit integral accumulation
        self.integral_max = 200.0  # Maximum integral value to prevent windup
        
        # Minimum dt to avoid division by zero
        self.min_dt = 0.001
        
        # Reset counter for periodic integral reset
        self.iteration_count = 0
        self.integral_reset_interval = 20  # Reset integral every N iterations

    def compute(self, error: float, dt: float, integral_reset_interval: int = None) -> float:
        """
        Compute PID control output.
        
        Args:
            error: Current error (pixels)
            dt: Time delta since last call (seconds)
            integral_reset_interval: Optional override for reset interval
            
        Returns:
            Control signal (angle in degrees, or 0.0 for straight when in dead zone)
        """
        if integral_reset_interval is not None:
            self.integral_reset_interval = integral_reset_interval
        
        # Ensure dt is valid
        if dt <= 0:
            dt = self.min_dt
        
        # Dead zone: if error is very small, go straight
        if abs(error) < self.tolerance:
            # Reset integral when in dead zone to prevent accumulation
            self.integral = 0.0
            self.prev_error = error
            return 0.0  # Go straight
        
        # Proportional term
        proportional = error
        
        # Integral term with anti-windup protection
        self.integral += error * dt
        
        # Clamp integral to prevent windup
        self.integral = max(min(self.integral, self.integral_max), -self.integral_max)
        
        # Periodic integral reset to prevent long-term drift
        self.iteration_count += 1
        if self.iteration_count % self.integral_reset_interval == 0:
            self.integral = 0.0
        
        # Derivative term with filtering to reduce noise sensitivity
        raw_derivative = (error - self.prev_error) / dt
        
        # Apply low-pass filter to derivative to reduce noise
        derivative = self.derivative_alpha * raw_derivative + (1 - self.derivative_alpha) * self.prev_derivative
        self.prev_derivative = derivative
        
        # Compute PID output
        control_signal = self.Kp * proportional + self.Ki * self.integral + self.Kd * derivative
        
        # Clamp output to valid range
        control_signal = max(min(control_signal, self.max_output), self.min_output)
        
        # Update previous error
        self.prev_error = error
        
        return control_signal
    
    def reset(self):
        """Reset PID controller state (useful when restarting or changing parameters)."""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_derivative = 0.0
        self.iteration_count = 0
    
    def get_parameters(self) -> dict:
        """Get current PID parameters."""
        return {
            'Kp': self.Kp,
            'Ki': self.Ki,
            'Kd': self.Kd,
            'tolerance': self.tolerance,
            'max_output': self.max_output,
            'min_output': self.min_output
        }
    
    def set_parameters(self, Kp: float = None, Ki: float = None, Kd: float = None):
        """
        Update PID parameters dynamically.
        
        Args:
            Kp: New proportional gain (None to keep current)
            Ki: New integral gain (None to keep current)
            Kd: New derivative gain (None to keep current)
        """
        if Kp is not None:
            self.Kp = Kp
        if Ki is not None:
            self.Ki = Ki
        if Kd is not None:
            self.Kd = Kd
        # Reset state when parameters change
        self.reset()

