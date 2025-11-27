"""
PID Controller Module - Implements PID control for lane following.

Following SRP: This module only handles PID control logic.
"""


class PIDController:
    """
    PID Controller responsible only for PID calculation, max angle limits, and deadband.
    """
    def __init__(self, Kp: float, Ki: float, Kd: float, max_angle: float = 30.0, deadband: float = 1.0):
        """
        Initialize the PID controller.
        
        Args:
            Kp: Proportional gain
            Ki: Integral gain
            Kd: Derivative gain
            max_angle: Maximum output angle in degrees (default: 30.0)
            deadband: Deadband angle in degrees - if output < deadband, return 0 (default: 1.0)
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_angle = max_angle
        self.deadband = deadband
        
        # Output limits (symmetric around 0)
        self.max_output = max_angle
        self.min_output = -max_angle
        
        # State variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_derivative = 0.0
        self.prev_output = 0.0  # For output smoothing
        
        # Derivative filter (low-pass filter to reduce noise)
        self.derivative_alpha = 0.7  # Filter coefficient (0-1, higher = less filtering)
        
        # Output smoothing factor (Exponential Moving Average)
        # 1.0 = no smoothing, 0.1 = heavy smoothing
        self.output_alpha = 0.8 
        
        # Anti-windup: limit integral accumulation
        self.integral_max = 200.0  # Maximum integral value to prevent windup
        
        # Minimum dt to avoid division by zero
        self.min_dt = 0.001

    def compute(self, error: float, dt: float) -> float:
        """
        Compute PID control output.
        
        Args:
            error: Current error (degrees - angular error)
            dt: Time delta since last call (seconds)
            
        Returns:
            Control signal (angle in degrees, clamped to max_angle, or 0.0 if within deadband)
        """
        # Ensure dt is valid
        if dt <= 0:
            dt = self.min_dt
        
        # Proportional term
        proportional = error
        
        # Integral term with anti-windup protection
        self.integral += error * dt
        
        # Clamp integral to prevent windup
        self.integral = max(min(self.integral, self.integral_max), -self.integral_max)
        
        # Derivative term with filtering to reduce noise sensitivity
        raw_derivative = (error - self.prev_error) / dt
        
        # Apply low-pass filter to derivative to reduce noise
        derivative = self.derivative_alpha * raw_derivative + (1 - self.derivative_alpha) * self.prev_derivative
        self.prev_derivative = derivative
        
        # Compute PID output
        raw_output = self.Kp * proportional + self.Ki * self.integral + self.Kd * derivative
        
        # Apply output smoothing (EMA) to prevent sudden jumps
        control_signal = self.output_alpha * raw_output + (1 - self.output_alpha) * self.prev_output
        self.prev_output = control_signal
        
        # Clamp output to valid range (max_angle)
        control_signal = max(min(control_signal, self.max_output), self.min_output)
        
        # Apply deadband: if output is within deadband, return 0
        if abs(control_signal) < self.deadband:
            control_signal = 0.0
            # We do NOT reset integral here anymore to avoid "sticking" when error grows slowly
        
        # Update previous error
        self.prev_error = error
        
        return control_signal
    
    def reset(self):
        """Reset PID controller state (useful when restarting or changing parameters)."""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_derivative = 0.0
        self.prev_output = 0.0
    
    def get_parameters(self) -> dict:
        """Get current PID parameters."""
        return {
            'Kp': self.Kp,
            'Ki': self.Ki,
            'Kd': self.Kd,
            'max_angle': self.max_angle,
            'deadband': self.deadband
        }
    
    def set_parameters(self, Kp: float = None, Ki: float = None, Kd: float = None, 
                     max_angle: float = None, deadband: float = None):
        """
        Update PID parameters dynamically.
        
        Args:
            Kp: New proportional gain (None to keep current)
            Ki: New integral gain (None to keep current)
            Kd: New derivative gain (None to keep current)
            max_angle: New max angle (None to keep current)
            deadband: New deadband (None to keep current)
        """
        if Kp is not None:
            self.Kp = Kp
        if Ki is not None:
            self.Ki = Ki
        if Kd is not None:
            self.Kd = Kd
        if max_angle is not None:
            self.max_angle = max_angle
            self.max_output = max_angle
            self.min_output = -max_angle
        if deadband is not None:
            self.deadband = deadband
        # Reset state when parameters change
        self.reset()
