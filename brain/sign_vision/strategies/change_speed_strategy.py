import time
from .base_strategy import SignStrategy


class ChangeSpeedStrategy(SignStrategy):
    """Generic strategy that sets the car to a target speed when a sign is detected.

    Used for signs like highway entry (increase speed) and highway exit (decrease speed).
    """

    def __init__(self, controller, lock, target_speed: int, cooldown=10.0,
                 min_confidence=0.6, activation_distance=2.0):
        """
        Args:
            target_speed: Speed (0-255) to set when the sign is detected.
            cooldown: Minimum seconds between activations.
        """
        super().__init__(controller, lock, min_confidence, activation_distance)
        self.target_speed = int(max(0, min(255, target_speed)))
        self.cooldown = cooldown
        self.last_activation_time = 0.0

    def execute(self, detection: dict) -> bool:
        if not self.validate_detection(detection):
            return False

        if time.time() - self.last_activation_time < self.cooldown:
            return False

        label = detection['class'].lower()
        confidence = detection['confidence']

        msg = f"{label.upper()} DETECTED! ({confidence:.2f}) - Setting speed to {self.target_speed}"
        print(f"[ChangeSpeedStrategy] {msg}")

        if self.controller.event_callback:
            self.controller.event_callback("sign_detected", {
                "label": label,
                "confidence": float(confidence),
                "message": msg,
            })

        sent = self.controller.command_sender.send_speed_command(self.target_speed)
        if not sent:
            print(f"[ChangeSpeedStrategy] Warning: failed to send speed command.")
            return False

        self.controller.update_current_speed(self.target_speed)
        with self.lock:
            self.controller.last_command = f"speed:{self.target_speed} ({label})"

        self.last_activation_time = time.time()
        return True

