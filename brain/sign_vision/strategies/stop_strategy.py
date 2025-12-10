import time
from .base_strategy import SignStrategy

class DefaultStopStrategy(SignStrategy):
    """Default strategy for handling STOP signs (Stop and wait)."""
    
    def __init__(self, controller, lock, cooldown=5.0, min_confidence=0.6, activation_distance=0.5):
        super().__init__(controller, lock, min_confidence, activation_distance)
        self.last_activation_time = 0
        self.cooldown = cooldown
        
    def execute(self, detection: dict) -> bool:
        """
        Stop the car, wait, and then resume (conceptually).
        Currently just stops.
        """
        # Check base conditions (confidence, distance)
        if not self.validate_detection(detection):
            return False
            
        current_time = time.time()
        
        # Check cooldown
        if current_time - self.last_activation_time < self.cooldown:
            return False
            
        label = detection['class'].lower()
        confidence = detection['confidence']
        
        msg = f"{label.upper()} DETECTED! ({confidence:.2f}) - Executing Default STOP Strategy"
        print(f"[DefaultStopStrategy] {msg}")
        
        # Report event
        if self.controller.event_callback:
            self.controller.event_callback("sign_detected", {
                "label": label, 
                "confidence": float(confidence), 
                "message": msg
            })
        
        # Save current speed before stopping
        with self.lock:
            if self.controller.current_speed > 0:
                self.controller.last_speed_before_stop = self.controller.current_speed
        
        # Send stop command
        self.controller.command_sender.send_speed_command(0)
        self.last_activation_time = current_time
        
        return True

