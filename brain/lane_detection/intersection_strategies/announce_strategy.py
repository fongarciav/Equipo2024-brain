import time
from .base_strategy import IntersectionStrategy


class AnnounceIntersectionStrategy(IntersectionStrategy):
    """Default non-invasive strategy: only reports intersection detections."""

    def __init__(self, event_callback=None):
        self.event_callback = event_callback

    def execute(self, intersection_state: dict):
        payload = {
            'source': 'lane_detector',
            'detected': bool(intersection_state.get('detected', False)),
            'valid_count': int(intersection_state.get('valid_count', 0)),
            'horizontal_count': int(intersection_state.get('horizontal_count', 0)),
            'raw_count': int(intersection_state.get('raw_count', 0)),
            'timestamp': time.time(),
        }
        print(f"[Intersection] Triggered: {payload}")
        if callable(self.event_callback):
            try:
                self.event_callback('intersection_detected', payload)
            except Exception as exc:
                print(f"[Intersection] event_callback error: {exc}")
