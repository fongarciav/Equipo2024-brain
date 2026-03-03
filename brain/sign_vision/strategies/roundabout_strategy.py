import time

from .base_strategy import SignStrategy


class RoundaboutStrategy(SignStrategy):
    """
    Strategy for handling roundabout signs.

    Behavior:
    - Temporarily reduce speed to a fixed safe value.
    - Temporarily tune lane-detector parameters so the reconstructed lane
      tends to stay closer to the inner curve of the roundabout.
    - Automatically restore original lane-detector parameters and speed
      after a short duration.
    """

    def __init__(
        self,
        controller,
        lock,
        cooldown: float = 10.0,
        min_confidence: float = 0.6,
        activation_distance: float = 3.0,
        roundabout_speed: int = 90,
        roundabout_duration: float = 4.0,
        roundabout_lane_width_px: int = 560,
        roundabout_straight_width_reduction: int = 20,
    ):
        super().__init__(controller, lock, min_confidence, activation_distance)
        self.cooldown = float(cooldown)
        self.roundabout_speed = int(max(0, min(255, roundabout_speed)))
        self.roundabout_duration = float(max(0.1, roundabout_duration))
        self.roundabout_lane_width_px = int(max(100, roundabout_lane_width_px))
        self.roundabout_straight_width_reduction = int(
            max(0, roundabout_straight_width_reduction)
        )

        self.last_activation_time: float = 0.0
        self.active_until: float = 0.0
        self.is_active: bool = False
        self._saved_lane_params: dict | None = None

    def _get_lane_detector(self):
        autopilot = getattr(self.controller, "autopilot_controller", None)
        if autopilot is None:
            return None
        return getattr(autopilot, "lane_detector", None)

    def _apply_lane_tuning(self) -> None:
        """
        Temporarily narrow reconstructed lane behavior for roundabout entry.
        """
        lane_detector = self._get_lane_detector()
        if lane_detector is None:
            return

        has_lane_width = hasattr(lane_detector, "LANE_WIDTH_PX")
        has_straight_reduction = hasattr(
            lane_detector, "STRAIGHT_LANE_WIDTH_REDUCTION"
        )
        if not has_lane_width or not has_straight_reduction:
            return

        # Save originals only once per active window.
        if self._saved_lane_params is None:
            self._saved_lane_params = {
                "LANE_WIDTH_PX": lane_detector.LANE_WIDTH_PX,
                "STRAIGHT_LANE_WIDTH_REDUCTION": lane_detector.STRAIGHT_LANE_WIDTH_REDUCTION,
            }

        lane_detector.LANE_WIDTH_PX = self.roundabout_lane_width_px
        lane_detector.STRAIGHT_LANE_WIDTH_REDUCTION = (
            self.roundabout_straight_width_reduction
        )

    def _restore_lane_tuning(self) -> None:
        lane_detector = self._get_lane_detector()
        if lane_detector is None:
            self._saved_lane_params = None
            return

        if self._saved_lane_params is not None:
            lane_detector.LANE_WIDTH_PX = self._saved_lane_params["LANE_WIDTH_PX"]
            lane_detector.STRAIGHT_LANE_WIDTH_REDUCTION = self._saved_lane_params[
                "STRAIGHT_LANE_WIDTH_REDUCTION"
            ]
        self._saved_lane_params = None

    def _deactivate_if_expired(self) -> None:
        if not self.is_active:
            return
        if time.time() < self.active_until:
            return

        self._restore_lane_tuning()
        self.is_active = False

        if self.controller.event_callback:
            self.controller.event_callback(
                "roundabout_mode_ended",
                {"duration_seconds": float(self.roundabout_duration)},
            )

    def on_loop(self) -> None:
        """
        Optional per-loop hook called by SignController.
        Keeps the strategy non-blocking and restores state on time.
        """
        self._deactivate_if_expired()

    def on_shutdown(self) -> None:
        """
        Restore tuned parameters if controller is stopped mid-roundabout.
        """
        if self.is_active:
            self._restore_lane_tuning()
            self.is_active = False

    def execute(self, detection: dict) -> bool:
        if not self.validate_detection(detection):
            return False

        now = time.time()
        if now - self.last_activation_time < self.cooldown:
            return False

        # Avoid unintentionally accelerating from standstill due to false positives.
        with self.lock:
            speed_before_roundabout = self.controller.current_speed

        if speed_before_roundabout <= 0:
            print("[RoundaboutStrategy] Car is not moving, skipping.")
            return False

        label = detection["class"].lower()
        confidence = detection["confidence"]

        msg = (
            f"{label.upper()} DETECTED! ({confidence:.2f}) "
            f"- roundabout mode for {self.roundabout_duration:.1f}s "
            f"(speed={self.roundabout_speed}, resume={speed_before_roundabout})"
        )
        print(f"[RoundaboutStrategy] {msg}")

        if self.controller.event_callback:
            self.controller.event_callback(
                "sign_detected",
                {
                    "label": label,
                    "confidence": float(confidence),
                    "message": msg,
                },
            )

        sent = self.controller.command_sender.send_speed_command(self.roundabout_speed)
        if not sent:
            print("[RoundaboutStrategy] Warning: failed to send roundabout speed.")
            return False

        with self.lock:
            self.controller.current_speed = self.roundabout_speed
            self.controller.last_command = (
                f"speed:{self.roundabout_speed} (roundabout)"
            )

        # Restore cruise speed after the roundabout window.
        self.controller.schedule_speed_resume(
            self.roundabout_duration, override_speed=speed_before_roundabout
        )

        self._apply_lane_tuning()
        self.is_active = True
        self.active_until = now + self.roundabout_duration
        self.last_activation_time = now

        if self.controller.event_callback:
            self.controller.event_callback(
                "roundabout_mode_started",
                {
                    "duration_seconds": float(self.roundabout_duration),
                    "roundabout_speed": int(self.roundabout_speed),
                    "resume_speed": int(speed_before_roundabout),
                    "lane_width_px": int(self.roundabout_lane_width_px),
                    "straight_width_reduction": int(
                        self.roundabout_straight_width_reduction
                    ),
                },
            )

        return True
