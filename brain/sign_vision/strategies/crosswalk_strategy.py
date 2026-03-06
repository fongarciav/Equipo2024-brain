import time
from .base_strategy import SignStrategy


class CrosswalkStrategy(SignStrategy):
    """Pedestrian: reduce speed 50% for 4 seconds, then restore."""

    def __init__(
        self,
        controller,
        lock,
        cooldown: float = 8.0,
        min_confidence: float = 0.6,
        activation_distance: float = 3.0,
    ):
        super().__init__(controller, lock, min_confidence, activation_distance)
        self.cooldown = cooldown
        self.slow_duration = 4.0
        self.last_activation_time: float = 0.0

    def execute(self, detection: dict) -> bool:
        if not self.validate_detection(detection):
            return False

        current_time = time.time()
        if current_time - self.last_activation_time < self.cooldown:
            return False

        label = detection["class"].lower()
        confidence = detection["confidence"]

        # Guardar la velocidad actual del auto para restaurarla después.
        with self.lock:
            saved_speed = self.controller.current_speed

        if saved_speed <= 0:
            print(f"[CrosswalkStrategy] Car is not moving, skipping.")
            return False

        # Reducir 50% durante 4 s, luego restaurar saved_speed
        slow_speed = int(max(0, min(255, round(saved_speed * 0.5))))

        msg = (
            f"{label.upper()} DETECTED! ({confidence:.2f}) "
            f"- 50% reducción: {saved_speed} -> {slow_speed} por {self.slow_duration:.0f}s, restore {saved_speed}"
        )
        print(f"[CrosswalkStrategy] {msg}")

        if self.controller.event_callback:
            self.controller.event_callback("sign_detected", {"label": label, "confidence": float(confidence), "message": msg})

        sent = self.controller.command_sender.send_speed_command(slow_speed)
        if not sent:
            print("[CrosswalkStrategy] Warning: failed to send slow-speed command.")
            return False

        with self.lock:
            self.controller.current_speed = slow_speed
            self.controller.last_command = f"speed:{slow_speed} (crosswalk -50%)"

        # Después de 4 s, inducir la velocidad guardada
        self.controller.schedule_speed_resume(self.slow_duration, override_speed=saved_speed)
        print(f"[CrosswalkStrategy] Resume en {self.slow_duration:.0f}s a {saved_speed}")

        if self.controller.event_callback:
            self.controller.event_callback("crosswalk_resume_scheduled", {
                "resume_in_seconds": float(self.slow_duration),
                "resume_speed": saved_speed,
                "slow_speed": slow_speed,
            })

        self.last_activation_time = current_time
        return True
