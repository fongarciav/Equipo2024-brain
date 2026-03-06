import threading
import time

from .base_strategy import SignStrategy


class RoundaboutHandsFreeStrategy(SignStrategy):
    """
    Roundabout strategy with a mostly hands-free flow:
    1) Passive wait (n1): let lane-following run normally.
    2) Straight entry (n2): force steering angle to 0 deg briefly.
    3) Left-line follow window: let lane-following hook to inner line.
    4) Right nudge (n3): brief forced right steering to point to exit lane.
    5) Return to normal lane-following automatically.
    """

    def __init__(
        self,
        controller,
        lock,
        cooldown: float = 10.0,
        min_confidence: float = 0.7,
        activation_distance: float = 2.0,
        n1_wait_passive_s: float = 0.8,
        n2_straight_entry_s: float = 0.9,
        n_left_follow_s: float = 2.0,
        n3_right_nudge_s: float = 1.0,
        right_nudge_angle_deg: float = -14.0,
    ):
        super().__init__(controller, lock, min_confidence, activation_distance)
        self.cooldown = float(max(0.0, cooldown))
        self.last_activation_time: float = 0.0

        self.n1_wait_passive_s = float(max(0.0, n1_wait_passive_s))
        self.n2_straight_entry_s = float(max(0.0, n2_straight_entry_s))
        self.n_left_follow_s = float(max(0.0, n_left_follow_s))
        self.n3_right_nudge_s = float(max(0.0, n3_right_nudge_s))
        self.right_nudge_angle_deg = float(right_nudge_angle_deg)

        self.is_running: bool = False
        self._worker_thread: threading.Thread | None = None

    def execute(self, detection: dict) -> bool:
        if not self.validate_detection(detection):
            return False

        autopilot = getattr(self.controller, "autopilot_controller", None)
        if autopilot is None:
            print("[RoundaboutHandsFreeStrategy] autopilot_controller unavailable, skipping.")
            return False

        now = time.time()
        if now - self.last_activation_time < self.cooldown:
            return False

        with self.lock:
            if self.is_running:
                return False
            self.is_running = True

        label = detection["class"].lower()
        confidence = detection["confidence"]
        msg = (
            f"{label.upper()} DETECTED! ({confidence:.2f}) - "
            "Starting hands-free roundabout sequence"
        )
        print(f"[RoundaboutHandsFreeStrategy] {msg}")
        if self.controller.event_callback:
            self.controller.event_callback(
                "sign_detected",
                {
                    "label": label,
                    "confidence": float(confidence),
                    "message": msg,
                },
            )

        self._worker_thread = threading.Thread(
            target=self._run_sequence,
            name="RoundaboutHandsFreeWorker",
            daemon=True,
        )
        self._worker_thread.start()

        self.last_activation_time = now
        return True

    def _emit_phase(self, phase: str):
        if self.controller.event_callback:
            self.controller.event_callback(
                "roundabout_phase_changed",
                {
                    "phase": phase,
                    "timestamp": time.time(),
                },
            )

    def _run_sequence(self):
        try:
            autopilot = getattr(self.controller, "autopilot_controller", None)
            if autopilot is None:
                return

            # Phase 1: passive wait (no override)
            self._emit_phase("passive_wait")
            time.sleep(self.n1_wait_passive_s)

            # Phase 2: force straight entry for a short window
            self._emit_phase("straight_entry")
            autopilot.start_timed_steering_override(
                steering_angle_deg=0.0,
                duration_s=self.n2_straight_entry_s,
                label="roundabout_entry_straight",
            )
            time.sleep(self.n2_straight_entry_s)

            # Phase 3: allow lane following to hook left inner line
            self._emit_phase("left_line_follow")
            time.sleep(self.n_left_follow_s)

            # Phase 4: short right nudge to aim for exit lane
            self._emit_phase("right_nudge")
            autopilot.start_timed_steering_override(
                steering_angle_deg=self.right_nudge_angle_deg,
                duration_s=self.n3_right_nudge_s,
                label="roundabout_exit_nudge",
            )
            time.sleep(self.n3_right_nudge_s)

            self._emit_phase("done")

        except Exception as exc:
            print(f"[RoundaboutHandsFreeStrategy] Error in sequence: {exc}")
            if self.controller.event_callback:
                self.controller.event_callback(
                    "roundabout_sequence_error",
                    {"error": str(exc), "timestamp": time.time()},
                )
        finally:
            with self.lock:
                self.is_running = False
