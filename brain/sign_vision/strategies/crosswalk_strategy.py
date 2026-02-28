import time
from .base_strategy import SignStrategy


class CrosswalkStrategy(SignStrategy):
    """Strategy for handling crosswalk signs: temporarily reduces speed, then restores it."""

    def __init__(
        self,
        controller,
        lock,
        cooldown: float = 10.0,
        min_confidence: float = 0.6,
        activation_distance: float = 3.0,
        slow_speed: int = 100,
        slow_duration: float = 6.0,
    ):
        super().__init__(controller, lock, min_confidence, activation_distance)
        self.cooldown = cooldown
        self.slow_speed = int(max(0, min(255, slow_speed)))
        self.slow_duration = float(slow_duration)
        self.last_activation_time: float = 0.0

    def execute(self, detection: dict) -> bool:
        if not self.validate_detection(detection):
            return False

        current_time = time.time()
        if current_time - self.last_activation_time < self.cooldown:
            return False

        label = detection["class"].lower()
        confidence = detection["confidence"]

        # Capture cruise speed before doing anything else.
        with self.lock:
            speed_before_slow = self.controller.current_speed

        if speed_before_slow <= 0:
            print(f"[CrosswalkStrategy] Car is not moving, skipping.")
            return False

        msg = (
            f"{label.upper()} DETECTED! ({confidence:.2f}) "
            f"- Slowing to {self.slow_speed} for {self.slow_duration:.1f}s, will resume at {speed_before_slow}"
        )
        print(f"[CrosswalkStrategy] {msg}")

        if self.controller.event_callback:
            self.controller.event_callback("sign_detected", {"label": label, "confidence": float(confidence), "message": msg})

        sent = self.controller.command_sender.send_speed_command(self.slow_speed)
        if not sent:
            print("[CrosswalkStrategy] Warning: failed to send slow-speed command.")
            return False

        with self.lock:
            self.controller.current_speed = self.slow_speed
            self.controller.last_command = f"speed:{self.slow_speed} (crosswalk slow)"

        # Schedule resume at the original cruise speed, not last_speed_before_stop.
        self.controller.schedule_speed_resume(self.slow_duration, override_speed=speed_before_slow)
        print(f"[CrosswalkStrategy] Resume scheduled in {self.slow_duration:.1f}s at speed {speed_before_slow}.")

        if self.controller.event_callback:
            self.controller.event_callback("crosswalk_resume_scheduled", {
                "resume_in_seconds": float(self.slow_duration),
                "resume_speed": speed_before_slow,
                "slow_speed": self.slow_speed,
            })

        self.last_activation_time = current_time
        return True
