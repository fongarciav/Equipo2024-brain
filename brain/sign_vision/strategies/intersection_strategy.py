import time
from .base_strategy import SignStrategy

class EnterIntersectionStrategy(SignStrategy):
    """
    Strategy for handling a priority/intersection sign.

    Does NOT pause the autopilot or send any manual commands.
    It simply registers the activation and lets the cooldown act as a
    shield so the sign does not re-trigger while the car crosses.
    The autopilot keeps running normally throughout the manoeuvre.
    """

    def __init__(self, controller, lock, cooldown=10.0, min_confidence=0.7, activation_distance=0.4):
        super().__init__(controller, lock, min_confidence, activation_distance)
        self.last_activation_time = 0
        self.cooldown = cooldown

    def execute(self, detection: dict) -> bool:
        """
        Acknowledge the intersection sign without taking any driving action.

        The cooldown window (default 10 s) ensures the strategy cannot
        re-trigger while the car is still crossing the intersection.
        """
        # Check base conditions (confidence, distance)
        if not self.validate_detection(detection):
            return False

        current_time = time.time()

        # Ignore if we recently handled this sign
        if current_time - self.last_activation_time < self.cooldown:
            return False

        label = detection['class'].lower()
        confidence = detection['confidence']

        msg = f"{label.upper()} DETECTED! ({confidence:.2f}) - Intersection acknowledged, autopilot continues"
        print(f"[EnterIntersectionStrategy] {msg}")

        if self.controller.event_callback:
            self.controller.event_callback("sign_detected", {
                "label": label,
                "confidence": float(confidence),
                "message": msg,
            })

        # Record activation time so the cooldown blocks re-triggers
        self.last_activation_time = current_time
        return True
