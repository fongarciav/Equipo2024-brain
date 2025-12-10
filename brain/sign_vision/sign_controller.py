"""
Sign Controller Module - Orchestrates sign detection and command sending.

Following SRP: This module orchestrates the interaction between the sign detector and the command sender.
"""

import threading
import time
import sys
from pathlib import Path

# Add brain directory to path to import shared modules
brain_dir = Path(__file__).parent.parent
if str(brain_dir) not in sys.path:
    sys.path.insert(0, str(brain_dir))

from command_sender import CommandSender
from sign_vision.sign_detector import SignDetector
from .strategies import DefaultStopStrategy, EnterIntersectionStrategy, IncreaseSpeedAndLaneWidthStrategy


class SignController:
    """Orchestrates sign detection and command sending."""
    
    def __init__(self, sign_detector: SignDetector, command_sender: CommandSender, event_callback=None, autopilot_controller=None):
        """
        Initialize the sign controller.
        
        Args:
            sign_detector: SignDetector instance for detecting signs
            command_sender: CommandSender instance for sending commands
            event_callback: Optional callback function(event_type, data) for reporting events
            autopilot_controller: Optional AutoPilotController instance to pause/resume lane following
        """
        self.sign_detector = sign_detector
        self.command_sender = command_sender
        self.event_callback = event_callback
        self.autopilot_controller = autopilot_controller
        
        self.is_running = False
        self.thread = None
        self.lock = threading.Lock()
        
        # State tracking
        self.last_command = None
        self.command_count = 0
        self.error_count = 0
        self.current_speed = 0
        self.last_speed_before_stop = 0  # Right now, default speed from esp32 at boot is zero, so if not last speed before stop is set, it will be zero
        
        # Cooldowns (to prevent spamming commands)
        self.last_stop_time = 0
        self.stop_cooldown = 5.0  # Seconds before stopping again
        
        # Initialize strategies
        self.strategies = {
            'stop': DefaultStopStrategy(self, self.lock),
            'no_entry': DefaultStopStrategy(self, self.lock, self.stop_cooldown),
            'onewayroad': IncreaseSpeedAndLaneWidthStrategy(self, self.lock)
        }
        
    def start(self):
        """Start the sign controller."""
        with self.lock:
            if self.is_running:
                return False
            
            self.is_running = True
            self.thread = threading.Thread(target=self._control_loop, daemon=True)
            self.thread.start()
            print("[SignController] Started")
            return True
    
    def stop(self):
        """Stop the sign controller."""
        with self.lock:
            if not self.is_running:
                return False
            
            self.is_running = False
            print("[SignController] Stopping...")
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
            print("[SignController] Stopped")
        
        return True
    
    def _control_loop(self):
        """Main control loop running in background thread."""
        while True:
            with self.lock:
                if not self.is_running:
                    break
            
            try:
                # 1. Get latest detections
                detections = self.sign_detector.get_detections()
                
                # 2. Process detections and decide action
                action_taken = False
                
                # Sort by confidence (highest first)
                detections.sort(key=lambda x: x['confidence'], reverse=True)
                
                for detection in detections:
                    label = detection['class'].lower()
                    confidence = detection['confidence']
                    
                    # Use dynamic confidence threshold from detector
                    threshold = self.sign_detector.confidence_threshold
                    
                    # --- LOGIC BASED ON PROVIDED CLASSES ---
                    
                    # Check if we have a strategy for this label
                    if label in self.strategies and confidence >= threshold:
                        strategy = self.strategies[label]
                        if strategy.execute(detection):
                            action_taken = True
                            break # Priority action taken
                        
                if action_taken:
                    with self.lock:
                        self.command_count += 1
                
                # Control loop rate (10Hz is usually enough for signs)
                time.sleep(0.1)
                
            except Exception as e:
                print(f"[SignController] Error in control loop: {e}")
                with self.lock:
                    self.error_count += 1
                time.sleep(1.0)

    def get_status(self) -> dict:
        """Get current status of the sign controller."""
        with self.lock:
            return {
                'is_running': self.is_running,
                'command_count': self.command_count,
                'error_count': self.error_count,
                'last_command': self.last_command,
                'current_speed': self.current_speed
            }

    def update_current_speed(self, speed: int):
        """
        Update the current speed tracking from external events (e.g. dashboard, serial).
        This allows the controller to know if the car is stopped or moving,
        and what speed to resume to.
        """
        with self.lock:
            self.current_speed = speed
            # If we are moving (speed > 0), remember it for resume
            if speed > 0:
                self.last_speed_before_stop = speed

