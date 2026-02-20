"""
AutoPilot Controller Module - Orchestrates lane detection, angle conversion, and command sending.

Following SRP: This module orchestrates the interaction between detector, converter, and sender.
"""

import threading
import time
from collections import deque

# Import lane following modules (relative imports since we're in lane_detection)
from .lane_detector import LaneDetector
from .pid_controller import PIDController
from .filter_controller import FilterController
from .angle_converter import AngleConverter
# Import shared resources (absolute imports - parent dir is in sys.path)
from command_sender import CommandSender
from camera.video_streamer import VideoStreamer


class AutoPilotController:
    """Orchestrates lane detection, angle conversion, and command sending."""
    
    def __init__(self, video_streamer: VideoStreamer, command_sender: CommandSender,
                 lane_detector: LaneDetector,
                 pid_kp: float = 0.06, pid_ki: float = 0.002, 
                 pid_kd: float = 0.02, max_angle: float = 30.0, deadband: float = 6.0,
                 max_change: float = 20.0,
                 hold_last_value_frames: int = 8,
                 safety_timeout_frames: int = 15,
                 max_steering_change_per_cycle: float = 8.0,
                 diagnostics_history_size: int = 300):
        """
        Initialize the auto-pilot controller.
        
        Args:
            video_streamer: VideoStreamer instance for getting frames
            command_sender: CommandSender instance for sending commands
            lane_detector: LaneDetector strategy instance for lane detection
            pid_kp: PID proportional gain
            pid_ki: PID integral gain
            pid_kd: PID derivative gain
            max_angle: Maximum steering angle in degrees (default: 30.0)
            deadband: Deadband angle in degrees (default: 6.0)
            max_change: Maximum allowed change in angle deviation between frames (default: 20.0)
            hold_last_value_frames: Number of frames to keep using last valid lane angle when detection is missing
            safety_timeout_frames: Number of consecutive missing frames before forcing safe steering mode
            max_steering_change_per_cycle: Max steering angle delta (deg) allowed per control cycle
            diagnostics_history_size: Number of recent diagnostic records to keep in memory
        """
        self.video_streamer = video_streamer
        self.command_sender = command_sender
        self.lane_detector = lane_detector
        self.angle_converter = AngleConverter()
        self.filter_controller = FilterController(max_change=max_change)
        
        # Initialize PID controller
        self.pid_controller = PIDController(
            Kp=pid_kp,
            Ki=pid_ki,
            Kd=pid_kd,
            max_angle=max_angle,
            deadband=deadband
        )
        
        # Store PID parameters for later updates
        self.pid_kp = pid_kp
        self.pid_ki = pid_ki
        self.pid_kd = pid_kd
        self.max_angle = max_angle
        self.deadband = deadband
        self.hold_last_value_frames = max(0, int(hold_last_value_frames))
        self.safety_timeout_frames = max(self.hold_last_value_frames, int(safety_timeout_frames))
        self.max_steering_change_per_cycle = max(0.0, float(max_steering_change_per_cycle))
        
        # Time tracking for PID dt calculation
        self.last_time = time.time()
        
        self.is_running = False
        self.is_paused = False
        self.thread = None
        self.lock = threading.Lock()
        
        # Statistics
        self.last_steering_angle = None
        self.last_servo_angle = None
        self.command_count = 0
        self.error_count = 0

        # Continuity and safety state
        self.last_valid_angle_deg = None
        self.frames_without_lane = 0
        self.last_sent_steering_angle = None

        # Runtime diagnostics ring buffer
        self.recent_diagnostics = deque(maxlen=max(10, int(diagnostics_history_size)))
        
        # Debug images storage
        self.last_debug_images = None
    
    def start(self):
        """Start the auto-pilot controller."""
        with self.lock:
            if self.is_running:
                return False
            
            self.is_running = True
            self.last_debug_images = None  # Clear stale images
            self.thread = threading.Thread(target=self._control_loop, daemon=True)
            self.thread.start()
            print("[AutoPilotController] Started")
            return True
    
    def stop(self):
        """Stop the auto-pilot controller."""
        with self.lock:
            if not self.is_running:
                return False
            
            self.is_running = False
            print("[AutoPilotController] Stopping...")
        
        # Wait for the control loop thread to finish (with timeout)
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
            if self.thread.is_alive():
                print("[AutoPilotController] Warning: Control loop thread did not stop within timeout")
            else:
                print("[AutoPilotController] Stopped")
        
        return True
    
    def pause(self):
        """Pause the auto-pilot controller (stop processing but keep thread alive)."""
        with self.lock:
            self.is_paused = True
            print("[AutoPilotController] Paused")

    def resume(self):
        """Resume the auto-pilot controller."""
        with self.lock:
            self.is_paused = False
            # Reset controllers to avoid jumps
            self.pid_controller.reset()
            self.filter_controller.reset()
            self.frames_without_lane = 0
            self.last_sent_steering_angle = None
            print("[AutoPilotController] Resumed")

    def _control_loop(self):
        """Main control loop running in background thread."""
        while True:
            # Check if still running at the start of each iteration
            with self.lock:
                if not self.is_running:
                    break
            
            try:
                # Get frame from video streamer
                frame = self.video_streamer.get_frame()
                
                # Check again after getting frame (stop might have been called)
                with self.lock:
                    if not self.is_running:
                        break
                
                if frame is None:
                    time.sleep(0.033)  # Wait ~30ms if no frame
                    continue

                # Get lane metrics from detector strategy
                # We run this even if paused to keep debug images active
                angle_deviation_deg, debug_images = self.lane_detector.get_lane_metrics(frame)
                
                # Store debug images for streaming
                with self.lock:
                    if debug_images is not None:
                        self.last_debug_images = debug_images
                
                # Check if paused
                is_paused = False
                with self.lock:
                    is_paused = self.is_paused

                if is_paused:
                    time.sleep(0.033)
                    continue
                
                # Handle no lanes detected
                raw_angle_deviation = angle_deviation_deg
                filtered_angle = None
                used_angle = None
                hold_active = False
                safe_mode_active = False

                if raw_angle_deviation is not None:
                    filtered_angle = self.filter_controller.filter(raw_angle_deviation)
                    if filtered_angle is not None:
                        used_angle = filtered_angle
                        with self.lock:
                            self.last_valid_angle_deg = filtered_angle
                            self.frames_without_lane = 0
                    else:
                        print(f"[Filter] Rejected outlier: {raw_angle_deviation}")
                        with self.lock:
                            self.frames_without_lane += 1
                else:
                    with self.lock:
                        self.frames_without_lane += 1

                if used_angle is None:
                    with self.lock:
                        current_missing = self.frames_without_lane
                        last_valid_angle = self.last_valid_angle_deg

                    if current_missing <= self.hold_last_value_frames and last_valid_angle is not None:
                        used_angle = last_valid_angle
                        hold_active = True
                    else:
                        # Neutral steering when no valid measurement is available.
                        used_angle = 0.0
                        safe_mode_active = True
                        if current_missing > self.safety_timeout_frames:
                            # Prolonged loss: clear dynamic state and keep neutral command.
                            self.pid_controller.reset()
                            self.filter_controller.reset()
                            with self.lock:
                                self.last_valid_angle_deg = None
                                self.last_sent_steering_angle = None
                
                # Calculate dt for PID
                current_time = time.time()
                dt = current_time - self.last_time
                if dt <= 0:
                    dt = 0.033  # Default to ~30 FPS if dt is invalid
                self.last_time = current_time
                
                # Check if still running before processing and sending commands
                with self.lock:
                    if not self.is_running:
                        break
                
                # Compute PID output (negate deviation for PID error)
                # Positive deviation (lane to right) → positive steering (turn right)
                pid_error = -used_angle
                #print(f"Original angle deviation: {angle_deviation_deg}")
                steering_angle = self.pid_controller.compute(pid_error, dt)
                #print(f"PID Controller: Steering angle: {steering_angle}")

                # Explicit slew-rate limiter on steering output to prevent abrupt changes.
                with self.lock:
                    prev_sent_steering = self.last_sent_steering_angle
                if prev_sent_steering is not None:
                    delta = steering_angle - prev_sent_steering
                    max_delta = self.max_steering_change_per_cycle
                    if delta > max_delta:
                        steering_angle = prev_sent_steering + max_delta
                    elif delta < -max_delta:
                        steering_angle = prev_sent_steering - max_delta
                
                # Check again before sending command (stop might have been called)
                with self.lock:
                    if not self.is_running:
                        break
                
                # Convert steering angle to servo angle
                servo_angle = self.angle_converter.convert(steering_angle, inverted=True)
                #print(f"Angle Converter: Servo angle: {servo_angle}")
                
                # Send command to ESP32 only if it changed (prevent UART spam)
                should_send = False
                with self.lock:
                    if self.last_servo_angle != servo_angle:
                        should_send = True
                
                if should_send:
                    success = self.command_sender.send_steering_command(servo_angle)
                    
                    # Update statistics
                    with self.lock:
                        self.last_steering_angle = steering_angle
                        self.last_servo_angle = servo_angle
                        if success:
                            self.command_count += 1
                        else:
                            self.error_count += 1
                else:
                    # Just update steering angle for dashboard, even if we didn't send command
                    with self.lock:
                        self.last_steering_angle = steering_angle

                with self.lock:
                    self.last_sent_steering_angle = steering_angle
                    self.recent_diagnostics.append({
                        'ts': current_time,
                        'raw_angle': raw_angle_deviation,
                        'filtered_angle': filtered_angle,
                        'used_angle': used_angle,
                        'hold_active': hold_active,
                        'safe_mode_active': safe_mode_active,
                        'frames_without_lane': self.frames_without_lane,
                        'steering_angle': steering_angle,
                        'servo_angle': servo_angle,
                        'command_sent': should_send
                    })
                
                # Control loop rate (~30 FPS)
                time.sleep(0.033)
                
            except Exception as e:
                print(f"[AutoPilotController] Error in control loop: {e}")
                self.error_count += 1
                time.sleep(0.1)
    
    def get_status(self) -> dict:
        """Get current status of the auto-pilot controller."""
        with self.lock:
            return {
                'is_running': self.is_running,
                'last_steering_angle': self.last_steering_angle,
                'last_servo_angle': self.last_servo_angle,
                'command_count': self.command_count,
                'error_count': self.error_count,
                'frames_without_lane': self.frames_without_lane,
                'hold_last_value_frames': self.hold_last_value_frames,
                'safety_timeout_frames': self.safety_timeout_frames,
                'max_steering_change_per_cycle': self.max_steering_change_per_cycle,
                'last_diagnostic': self.recent_diagnostics[-1] if self.recent_diagnostics else None
            }
    
    def update_pid_parameters(self, kp: float = None, ki: float = None, kd: float = None, 
                             max_angle: float = None, deadband: float = None):
        """
        Update PID parameters dynamically.
        
        Args:
            kp: New proportional gain (None to keep current)
            ki: New integral gain (None to keep current)
            kd: New derivative gain (None to keep current)
            max_angle: New max angle (None to keep current)
            deadband: New deadband (None to keep current)
        """
        with self.lock:
            if kp is not None:
                self.pid_kp = kp
            if ki is not None:
                self.pid_ki = ki
            if kd is not None:
                self.pid_kd = kd
            if max_angle is not None:
                self.max_angle = max_angle
            if deadband is not None:
                self.deadband = deadband
            
            # Update PID controller parameters
            self.pid_controller.set_parameters(
                Kp=kp, Ki=ki, Kd=kd, max_angle=max_angle, deadband=deadband
            )
    
    def get_pid_parameters(self) -> dict:
        """Get current PID parameters."""
        with self.lock:
            pid_params = self.pid_controller.get_parameters()
            return {
                'kp': pid_params['Kp'],
                'ki': pid_params['Ki'],
                'kd': pid_params['Kd'],
                'max_angle': pid_params['max_angle'],
                'deadband': pid_params['deadband']
            }
    
    def get_debug_image(self, image_key: str):
        """
        Get a debug image by key.
        
        Args:
            image_key: Key of the debug image ('bird_view_lines', 'sliding_windows', 'final_result', etc.)
            
        Returns:
            Image (numpy array) or None if not available
        """
        with self.lock:
            if self.last_debug_images and image_key in self.last_debug_images:
                return self.last_debug_images[image_key].copy()
        return None

