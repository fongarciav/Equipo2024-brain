import time
from .intersection_strategies import AnnounceIntersectionStrategy


class IntersectionEventController:
    """Debounces lane-based intersection detections and triggers a strategy."""

    def __init__(self, strategy=None, consecutive_frames=3, cooldown_s=2.5):
        self.strategy = strategy or AnnounceIntersectionStrategy()
        self.consecutive_frames = max(1, int(consecutive_frames))
        self.cooldown_s = max(0.0, float(cooldown_s))

        self._detected_streak = 0
        self._last_trigger_time = 0.0
        self.last_state = {
            'detected': False,
            'valid_count': 0,
            'horizontal_count': 0,
            'raw_count': 0,
            'triggered': False,
            'streak': 0,
        }

    def process(self, intersection_state: dict):
        state = intersection_state or {}
        detected = bool(state.get('detected', False))
        now = time.time()

        if detected:
            self._detected_streak += 1
        else:
            self._detected_streak = 0

        triggered = False
        if detected and self._detected_streak >= self.consecutive_frames:
            if (now - self._last_trigger_time) >= self.cooldown_s:
                self.strategy.execute(state)
                self._last_trigger_time = now
                triggered = True

        self.last_state = {
            'detected': detected,
            'valid_count': int(state.get('valid_count', 0)),
            'horizontal_count': int(state.get('horizontal_count', 0)),
            'raw_count': int(state.get('raw_count', 0)),
            'triggered': triggered,
            'streak': int(self._detected_streak),
            'cooldown_remaining_s': max(0.0, self.cooldown_s - (now - self._last_trigger_time)),
        }
        return triggered
