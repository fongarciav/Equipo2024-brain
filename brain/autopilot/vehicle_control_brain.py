"""
Vehicle Control Brain Module - Orchestrates lane detection and PID control.

This module acts as the main orchestrator, combining lane detection with PID control.
"""

import time
import cv2
from .lane_detector import MarcosLaneDetector_Advanced
from .pid_controller import PIDController


class VehicleControlBrain:
    """
    Main orchestrator that combines lane detection with PID control.
    """
    
    def __init__(self, lane_threshold: int = 180, Kp: float = 0.06, Ki: float = 0.002, 
                 Kd: float = 0.02, max_angle: float = 30.0, deadband: float = 6.0):
        """
        Initialize the Vehicle Control Brain.
        
        Args:
            lane_threshold: Image processing threshold for lane detection
            Kp: PID proportional gain
            Ki: PID integral gain
            Kd: PID derivative gain
            max_angle: Maximum steering angle in degrees (default: 30.0)
            deadband: Deadband angle in degrees (default: 6.0)
        """
        # Initialize lane detector (no PID code)
        self.detector = MarcosLaneDetector_Advanced(threshold=lane_threshold)
        
        # Initialize PID controller
        self.pid_controller = PIDController(
            Kp=Kp,
            Ki=Ki,
            Kd=Kd,
            max_angle=max_angle,
            deadband=deadband
        )
        
        # Time tracking for PID dt calculation
        self.last_time = time.time()
        
        # Store parameters for later updates
        self.lane_threshold = lane_threshold
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_angle = max_angle
        self.deadband = deadband
    
    def process_frame(self, frame):
        """
        Process a frame and return steering angle and debug images.
        
        Args:
            frame: Input frame from camera
            
        Returns:
            tuple: (steering_angle, debug_images)
                - steering_angle: Final steering angle in degrees (from PID)
                - debug_images: Dictionary of debug images
        """
        # Get lane metrics from detector
        angle_desviacion_deg, debug_images = self.detector.get_lane_metrics(frame)
        
        # Handle None case (no lanes found)
        if angle_desviacion_deg is None:
            # Reset PID integral when no lanes detected
            self.pid_controller.reset()
            return 0.0, debug_images
        
        # Calculate dt for PID
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 0.033  # Default to ~30 FPS if dt is invalid
        self.last_time = current_time
        
        # Feed NEGATED deviation angle to PID
        # Positive deviation (lane to right) → positive steering (turn right)
        # PID is designed for setpoint - current_value, so we negate the deviation
        pid_error = -angle_desviacion_deg
        
        # Compute PID output
        steering_angle = self.pid_controller.compute(pid_error, dt)
        
        # Update debug images with PID information
        if debug_images and "final_result" in debug_images:
            result = debug_images["final_result"]
            cv2.putText(result, f'PID Angle: {steering_angle:.2f} deg', (30, 190), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        if debug_images and "bird_view_lines" in debug_images:
            bird_view = debug_images["bird_view_lines"]
            cv2.putText(bird_view, f'PID Angle: {steering_angle:.2f} deg', (10, 180), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return steering_angle, debug_images
    
    def update_pid_parameters(self, Kp: float = None, Ki: float = None, Kd: float = None,
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
        if deadband is not None:
            self.deadband = deadband
        
        # Update PID controller parameters
        self.pid_controller.set_parameters(
            Kp=Kp, Ki=Ki, Kd=Kd, max_angle=max_angle, deadband=deadband
        )
    
    def get_pid_parameters(self) -> dict:
        """Get current PID parameters."""
        return self.pid_controller.get_parameters()
    
    def reset_pid(self):
        """Reset PID controller state."""
        self.pid_controller.reset()

