import time
from .base_strategy import SignStrategy


class CrosswalkStrategy(SignStrategy):
    """Strategy for handling crosswalk signs: slows down and resumes previous speed."""

    def __init__(
        self,
        controller,
        lock,
        cooldown: float = 10.0,
        min_confidence: float = 0.6,
        activation_distance: float = 3.0,
        slow_speed: int = 100,
        slow_duration: float = 3.0,
    ):
        """
        Args:
            controller: The SignController instance.
            lock: Threading lock shared with the controller.
            cooldown: Minimum seconds between activations.
            min_confidence: Minimum detection confidence to react.
            activation_distance: Maximum distance (m) at which the sign triggers.
            slow_speed: Speed value (0–255) to apply when crossing.
            slow_duration: Seconds to maintain the reduced speed before resuming.
        """
        super().__init__(controller, lock, min_confidence, activation_distance)
        self.cooldown = cooldown
        self.slow_speed = int(max(0, min(255, slow_speed)))
        self.slow_duration = float(slow_duration)
        self.last_activation_time: float = 0.0

    def execute(self, detection: dict) -> bool:
        """
        Reduce speed when a crosswalk sign is detected, then restore previous speed
        after slow_duration seconds using the controller's non-blocking resume mechanism.

        Returns:
            True if the slow-down command was sent successfully, False otherwise.
        """
        if not self.validate_detection(detection):
            return False

        current_time = time.time()
        if current_time - self.last_activation_time < self.cooldown:
            return False

        label = detection["class"].lower()
        confidence = detection["confidence"]

        msg = (
            f"{label.upper()} DETECTED! ({confidence:.2f}) "
            f"- Slowing to {self.slow_speed} for {self.slow_duration:.1f}s"
        )
        print(f"[CrosswalkStrategy] {msg}")

        if self.controller.event_callback:
            self.controller.event_callback(
                "sign_detected",
                {
                    "label": label,
                    "confidence": float(confidence),
                    "message": msg,
                },
            )

        # Send the reduced-speed command.
        sent = self.controller.command_sender.send_speed_command(self.slow_speed)
        if not sent:
            print("[CrosswalkStrategy] Warning: failed to send slow-speed command.")
            return False

        # Trigger last_speed_before_stop save via update_current_speed before
        # overwriting current_speed with slow_speed.
        self.controller.update_current_speed(0)
        with self.lock:
            self.controller.current_speed = self.slow_speed
            self.controller.last_command = f"speed:{self.slow_speed} (crosswalk slow)"

        # Schedule automatic speed restoration (non-blocking).
        resume_speed = self.controller.schedule_speed_resume(self.slow_duration)
        print(
            f"[CrosswalkStrategy] Resume scheduled in {self.slow_duration:.1f}s "
            f"at speed {resume_speed}."
        )

        if self.controller.event_callback:
            self.controller.event_callback(
                "crosswalk_resume_scheduled",
                {
                    "resume_in_seconds": float(self.slow_duration),
                    "resume_speed": int(resume_speed),
                    "slow_speed": self.slow_speed,
                },
            )

        self.last_activation_time = current_time
        return True
