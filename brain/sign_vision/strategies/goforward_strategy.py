import time
from .base_strategy import SignStrategy

class GoForwardStrategy(SignStrategy):
    """Strategy for simply moving forward at a specific speed."""
    
    def __init__(self, controller, lock, cooldown=5.0, min_confidence=0.7, activation_distance=1.0):
        super().__init__(controller, lock, min_confidence, activation_distance)
        self.last_activation_time = 0
        self.cooldown = cooldown

    def execute(self, detection: dict) -> bool:
        """
        Execute logic for going forward.
        Steps:
        1. Set speed to 230
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
        
        msg = f"{label.upper()} DETECTED! ({confidence:.2f}) - Increasing Speed"
        print(f"[GoForwardStrategy] {msg}")
        
        # Report event
        if self.controller.event_callback:
            self.controller.event_callback("sign_detected", {
                "label": label, 
                "confidence": float(confidence), 
                "message": msg
            })
            
        # Execute Action
        try:
            SPEED = 230
            print(f"[GoForwardStrategy] Setting speed to {SPEED}")
            
            # Send speed command
            success = self.controller.command_sender.send_speed_command(SPEED)
            
            if success:
                # Update controller's current speed tracker if successful
                self.controller.current_speed = SPEED
                
        except Exception as e:
            print(f"[GoForwardStrategy] Error executing action: {e}")
        
        self.last_activation_time = time.time()
        return True

