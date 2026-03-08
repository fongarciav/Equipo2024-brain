import time
from .base_strategy import IntersectionStrategy


class FirstIntersectionLeftTurnStrategy(IntersectionStrategy):
    """Execute a one-time left turn override on the first detected intersection."""

    def __init__(self, autopilot_controller, event_callback=None, duration_s=3.0, max_angle_ratio=0.75):
        self.autopilot_controller = autopilot_controller
        self.event_callback = event_callback
        self.duration_s = max(0.1, float(duration_s))
        self.duration_straight = 2.15  # Duration to go straight before turning
        self.max_angle_ratio = max(0.0, min(1.0, float(max_angle_ratio)))
        self._done = False

    def execute(self, intersection_state: dict):
        if self._done:
            return

        # Avanza recto
        self.autopilot_controller.start_timed_steering_override(
            steering_angle_deg=0.0,
            duration_s=self.duration_straight,
            label='first_intersection_straight'
        )

        # Espera a que termine el período recto antes de girar
        time.sleep(self.duration_straight)

        # Gira a la izquierda con un ángulo máximo durante un tiempo limitado
        target_angle = float(self.autopilot_controller.max_angle) * self.max_angle_ratio
        self.autopilot_controller.start_timed_steering_override(
            steering_angle_deg=target_angle,
            duration_s=self.duration_s,
            label='first_intersection_left_turn'
        )

        payload = {
            'source': 'lane_detector',
            'action': 'first_left_turn',
            'target_steering_angle_deg': target_angle,
            'duration_s': self.duration_s,
            'timestamp': time.time(),
            'intersection_state': {
                'detected': bool(intersection_state.get('detected', False)),
                'valid_count': int(intersection_state.get('valid_count', 0)),
                'horizontal_count': int(intersection_state.get('horizontal_count', 0)),
                'raw_count': int(intersection_state.get('raw_count', 0)),
            }
        }
        print(f"[IntersectionStrategy] Executed first left turn: {payload}")
        if callable(self.event_callback):
            try:
                self.event_callback('intersection_first_left_turn_executed', payload)
            except Exception as exc:
                print(f"[IntersectionStrategy] event_callback error: {exc}")

        self._done = True
