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
    
    def __init__(self, sign_detector: SignDetector, command_sender: CommandSender, event_callback=None):
        """
        Initialize the sign controller.
        
        Args:
            sign_detector: SignDetector instance for detecting signs
            command_sender: CommandSender instance for sending commands
            event_callback: Optional callback function(event_type, data) for reporting events
        """
        self.sign_detector = sign_detector
        self.command_sender = command_sender
        self.event_callback = event_callback
        
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
                    
                    # Priority 1: STOP Signals (stop sign, red light, no entry)
                    if label in ['stop', 'lightred', 'no_entry'] and confidence >= threshold:
                        current_time = time.time()
                        if current_time - self.last_stop_time > self.stop_cooldown:
                            msg = f"{label.upper()} DETECTED! ({confidence:.2f})"
                            print(f"[SignController] {msg}")
                            
                            # Report event
                            if self.event_callback:
                                self.event_callback("sign_detected", {
                                    "label": label, 
                                    "confidence": float(confidence), 
                                    "message": msg
                                })
                            
                            # Save current speed before stopping
                            with self.lock:
                                if self.current_speed > 0:
                                    self.last_speed_before_stop = self.current_speed
                            
                            self.command_sender.send_speed_command(0)
                            self.last_stop_time = current_time
                            action_taken = True
                            break # Priority action taken
                    
                    # Priority 2: Caution / Slow Down (crosswalk, roundabout, yellow light)
                    elif label in ['crosswalk', 'roundabout', 'lightyellow'] and confidence >= threshold:
                        msg = f"{label.upper()} DETECTED! ({confidence:.2f})"
                        print(f"[SignController] {msg}")
                        
                        # Report event
                        if self.event_callback:
                            self.event_callback("sign_detected", {
                                "label": label, 
                                "confidence": float(confidence), 
                                "message": msg
                            })
                        
                        # Placeholder: Implement slow down logic here
                        # self.command_sender.send_speed_command(10) 
                        pass
                        
                    # Priority 3: Go / Resume (green light, priority)
                    elif label in ['lightgreen', 'priority', 'highway_entry'] and confidence >= threshold:
                        # Only resume if we are stopped (or very slow) AND we have a previous speed
                        if self.current_speed == 0:
                            with self.lock:
                                target_speed = self.last_speed_before_stop
                            
                            # Only resume if we have a valid previous speed
                            if target_speed > 0:
                                msg = f"{label.upper()} DETECTED! Resuming to {target_speed} ({confidence:.2f})"
                                print(f"[SignController] {msg}")
                                
                                # Report event
                                if self.event_callback:
                                    self.event_callback("sign_detected", {
                                        "label": label, 
                                        "confidence": float(confidence), 
                                        "message": msg
                                    })
                                
                                # Soft Resume: Start from (Target - 10) and ramp up
                                start_speed = max(0, target_speed - 10)
                                
                                # Ramp up logic
                                current = start_speed
                                while current <= target_speed:
                                    self.command_sender.send_speed_command(current)
                                    current += 1
                                    time.sleep(0.05) # 50ms delay between steps
                                
                                action_taken = True
                                # We assume command works and update locally to prevent spamming (actual update comes from serial event)
                                with self.lock:
                                    self.current_speed = target_speed
                            else:
                                # No previous speed recorded, don't resume
                                msg = f"{label.upper()} DETECTED! No previous speed to resume to ({confidence:.2f})"
                                print(f"[SignController] {msg}")
                                
                                # Report event
                                if self.event_callback:
                                    self.event_callback("sign_detected", {
                                        "label": label, 
                                        "confidence": float(confidence), 
                                        "message": msg
                                    })
                        
                    # Other classes: parking, onewayroad, trafficlight, highway_exit
                        
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

