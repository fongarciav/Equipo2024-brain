import time
from .base_strategy import SignStrategy

class EnterIntersectionStrategy(SignStrategy):
    """Strategy for entering an intersection (disables lane following temporarily)."""
    
    def __init__(self, controller, lock, cooldown=10.0, min_confidence=0.7, activation_distance=0.5):
        super().__init__(controller, lock, min_confidence, activation_distance)
        self.last_activation_time = 0
        self.cooldown = cooldown

    def execute(self, detection: dict) -> bool:
        """
        Execute logic for entering an intersection.
        Steps:
        1. Pause Autopilot (Lane Detector)
        2. Execute manual movement sequence
        3. Resume Autopilot
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
        
        msg = f"{label.upper()} DETECTED! ({confidence:.2f}) - Entering Intersection Mode"
        print(f"[EnterIntersectionStrategy] {msg}")
        
        # Report event
        if self.controller.event_callback:
            self.controller.event_callback("sign_detected", {
                "label": label, 
                "confidence": float(confidence), 
                "message": msg
            })
            
        # 1. Pause Autopilot (Lane Following)
        if hasattr(self.controller, 'autopilot_controller') and self.controller.autopilot_controller:
            self.controller.autopilot_controller.pause()
            print("[EnterIntersectionStrategy] Autopilot paused")
        else:
            print("[EnterIntersectionStrategy] Warning: Autopilot controller not available, cannot pause lane following")

        # 2. Execute Manual Sequence
        # Sequence: Go Forward (1.5s) -> Turn Right (3.5s) -> Total 5s
        try:
            print("[EnterIntersectionStrategy] Executing manual sequence: Forward 1.5s then Right 3.5s")
            
            # SERVO Constants from angle_converter.py: CENTER=105, RIGHT=50, LEFT=160
            FORWARD_ANGLE = 105
            RIGHT_LEFT_ANGLE = 140 # Close to 50 (hard right)
            SPEED = 235

            # Use current speed if sensible, otherwise default
            if self.controller.current_speed > 0:
                SPEED = self.controller.current_speed

            # Phase 1: Enter Intersection (Forward)
            self.controller.command_sender.send_steering_command(RIGHT_LEFT_ANGLE)
            self.controller.command_sender.send_speed_command(SPEED)
            time.sleep(2)
            self.controller.command_sender.send_speed_command(SPEED)
            time.sleep(2)
            
        except Exception as e:
            print(f"[EnterIntersectionStrategy] Error during manual sequence: {e}")
        
        # 3. Resume Autopilot
        if hasattr(self.controller, 'autopilot_controller') and self.controller.autopilot_controller:
            self.controller.autopilot_controller.resume()
            print("[EnterIntersectionStrategy] Autopilot resumed")
        
        self.last_activation_time = time.time()
        return True
