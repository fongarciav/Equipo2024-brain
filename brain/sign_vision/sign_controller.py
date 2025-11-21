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


class SignController:
    """Orchestrates sign detection and command sending."""
    
    def __init__(self, sign_detector: SignDetector, command_sender: CommandSender):
        """
        Initialize the sign controller.
        
        Args:
            sign_detector: SignDetector instance for detecting signs
            command_sender: CommandSender instance for sending commands
        """
        self.sign_detector = sign_detector
        self.command_sender = command_sender
        
        self.is_running = False
        self.thread = None
        self.lock = threading.Lock()
        
        # State tracking
        self.last_command = None
        self.command_count = 0
        self.error_count = 0
        
        # Cooldowns (to prevent spamming commands)
        self.last_stop_time = 0
        self.stop_cooldown = 5.0  # Seconds before stopping again
        
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
                    
                    # Example Logic - CUSTOMIZE BASED ON YOUR MODEL LABELS
                    if 'stop' in label and confidence > 0.7:
                        current_time = time.time()
                        if current_time - self.last_stop_time > self.stop_cooldown:
                            print(f"[SignController] STOP SIGN DETECTED! ({confidence:.2f})")
                            self.command_sender.send_speed_command(0)
                            self.last_stop_time = current_time
                            action_taken = True
                            break # Priority action taken
                            
                    elif '50' in label: # Speed Limit 50
                        # self.command_sender.send_speed_command(50)
                        pass
                        
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
                'last_command': self.last_command
            }

