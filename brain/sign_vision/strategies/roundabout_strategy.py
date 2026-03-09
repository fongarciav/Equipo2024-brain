import time

from .base_strategy import SignStrategy


class RoundaboutStrategy(SignStrategy):
    """
    Strategy for handling roundabout signs.

    Behavior:
    - Temporarily reduce speed to a fixed safe value.
    - Run a staged maneuver with independent timers.
    - Automatically restore original lane-detector parameters and speed
      when all stages are completed.
    """

    def __init__(
        self,
        controller,
        lock,
        cooldown: float = 10.0,
        min_confidence: float = 0.6,
        activation_distance: float = 3.0,
        roundabout_speed: int = 60,
        pre_entry_duration: float = 5.0,
        lane_tuning_duration: float = 15.0,
        right_turn_duration: float = 3.0,
        roundabout_lane_width_px: int = 400,
        roundabout_straight_width_reduction: int = 0,
        right_turn_servo_angle: int = 50,
    ):
        super().__init__(controller, lock, min_confidence, activation_distance)
        self.cooldown = float(cooldown)
        self.roundabout_speed = int(max(0, min(255, roundabout_speed)))
        self.pre_entry_duration = float(max(0.1, pre_entry_duration))
        self.lane_tuning_duration = float(max(0.1, lane_tuning_duration))
        self.right_turn_duration = float(max(0.1, right_turn_duration))
        self.roundabout_lane_width_px = int(max(100, roundabout_lane_width_px))
        self.roundabout_straight_width_reduction = int(
            max(0, roundabout_straight_width_reduction)
        )
        self.right_turn_servo_angle = int(max(0, min(180, right_turn_servo_angle)))

        self.last_activation_time: float = 0.0
        self.is_active: bool = False
        self._saved_lane_params: dict | None = None
        self._entry_started_at: float = 0.0
        self._saved_speed_before_roundabout: int | None = None

    @property
    def total_duration(self) -> float:
        return (
            self.pre_entry_duration
            + self.lane_tuning_duration
            + self.right_turn_duration
        )

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

        now = time.time()
        elapsed = now - self._entry_started_at
        phase_1_end = self.pre_entry_duration
        phase_2_end = self.pre_entry_duration + self.lane_tuning_duration
        phase_3_end = self.total_duration

        if elapsed < phase_1_end:
            return

        if elapsed < phase_2_end:
            self._apply_lane_tuning()
            return

        if elapsed < phase_3_end:
            self._restore_lane_tuning()
            sent = self.controller.command_sender.send_steering_command(
                self.right_turn_servo_angle
            )
            if not sent:
                print("[RoundaboutStrategy] Warning: failed to send right turn steer.")
            return

        self._restore_lane_tuning()
        speed_to_restore = self._saved_speed_before_roundabout
        if speed_to_restore is not None:
            sent = self.controller.command_sender.send_speed_command(speed_to_restore)
            if sent:
                with self.lock:
                    self.controller.current_speed = speed_to_restore
                    self.controller.last_command = (
                        f"speed:{speed_to_restore} (roundabout_resume)"
                    )
            else:
                print("[RoundaboutStrategy] Warning: failed to restore speed.")

        self._saved_speed_before_roundabout = None
        self.is_active = False

        if self.controller.event_callback:
            self.controller.event_callback(
                "roundabout_mode_ended",
                {"duration_seconds": float(self.total_duration)},
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
            self._saved_speed_before_roundabout = None

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
            f"- roundabout mode for {self.total_duration:.1f}s "
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

        self.is_active = True
        self._entry_started_at = now
        self._saved_speed_before_roundabout = speed_before_roundabout
        self.last_activation_time = now

        if self.controller.event_callback:
            self.controller.event_callback(
                "roundabout_mode_started",
                {
                    "duration_seconds": float(self.total_duration),
                    "pre_entry_seconds": float(self.pre_entry_duration),
                    "lane_tuning_seconds": float(self.lane_tuning_duration),
                    "right_turn_seconds": float(self.right_turn_duration),
                    "roundabout_speed": int(self.roundabout_speed),
                    "resume_speed": int(speed_before_roundabout),
                    "lane_width_px": int(self.roundabout_lane_width_px),
                    "straight_width_reduction": int(
                        self.roundabout_straight_width_reduction
                    ),
                    "right_turn_servo_angle": int(self.right_turn_servo_angle),
                },
            )

        return True
