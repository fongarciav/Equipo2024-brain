import time
from .base_strategy import SignStrategy

class DefaultStopStrategy(SignStrategy):
    """Default strategy for handling STOP signs (Stop and wait)."""
    
    def __init__(
        self,
        controller,
        lock,
        cooldown=5.0,
        min_confidence=0.6,
        activation_distance=0.4,
        stop_duration=3.0
    ):
        super().__init__(controller, lock, min_confidence, activation_distance)
        self.last_activation_time = 0
        self.cooldown = cooldown
        self.stop_duration = stop_duration
        
    def execute(self, detection: dict) -> bool:
        """
        Stop the car and schedule a non-blocking resume after stop_duration.
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
        
        # Send stop command
        stop_sent = self.controller.command_sender.send_speed_command(0)
        if stop_sent:
            # update_current_speed maneja last_speed_before_stop automáticamente:
            # si speed > 0 lo guarda, si speed == 0 lo conserva intacto.
            self.controller.update_current_speed(0)
            with self.lock:
                self.controller.last_command = "speed:0 (stop)"
        else:
            print("[DefaultStopStrategy] Warning: failed to send STOP command")
            return False

        resume_speed = self.controller.schedule_speed_resume(self.stop_duration)
        print(
            f"[DefaultStopStrategy] Resume scheduled in {self.stop_duration:.1f}s "
            f"at speed {resume_speed}"
        )
        if self.controller.event_callback:
            self.controller.event_callback("stop_resume_scheduled", {
                "resume_in_seconds": float(self.stop_duration),
                "resume_speed": int(resume_speed)
            })

        self.last_activation_time = current_time
        
        return True
