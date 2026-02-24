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
        activation_distance=3.0,
        stop_duration=3.0,
        k=0.8,
        offset=0.25
    ):
        super().__init__(controller, lock, min_confidence, activation_distance)
        self.last_activation_time = 0
        self.cooldown = cooldown
        self.stop_duration = stop_duration
        self.k = float(k)
        self.offset = float(offset)
        # Keep conversion aligned with CommandSender.SPEED_MM_S_MAX = 500
        self.speed_mm_s_max = 500.0
        
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

        distance = detection.get('distance')
        if distance is None:
            print("[STOP] distance=None -> skipping STOP")
            return False

        with self.lock:
            current_speed_ui = int(self.controller.current_speed)

        v_mm_s = (current_speed_ui / 255.0) * self.speed_mm_s_max
        v_m_s = v_mm_s / 1000.0
        braking_distance = (self.k * v_m_s) + self.offset

        print(
            f"[STOP] v_ui={current_speed_ui}, "
            f"v_m_s={v_m_s:.3f}, "
            f"distance={float(distance):.3f}, "
            f"braking_distance={braking_distance:.3f}"
        )

        if float(distance) > braking_distance:
            print("[STOP] Sign is farther than braking distance -> skipping STOP")
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
        stop_sent = self.controller.command_sender.send_speed_command(0)
        if stop_sent:
            with self.lock:
                self.controller.current_speed = 0
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

