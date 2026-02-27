import threading
import time

from .base_strategy import SignStrategy
from command_sender import SERVO_CENTER, SERVO_LEFT, SERVO_RIGHT


class ParkingStrategy(SignStrategy):
    """
    Strategy for parallel parking to the left using a timed phase sequence.

    Phases:
        APPROACH  → Drive forward slowly to position alongside the parking spot.
        BACK_IN   → Reverse with full-left steering to swing the rear into the spot.
        ALIGN     → Reverse with full-right steering to straighten the car.
        CENTER    → Creep forward to center the car in the spot.
        STOP      → Halt and mark the maneuver as complete.

    The maneuver runs in a background daemon thread so the SignController
    control loop is never blocked.  A 30-second cooldown prevents re-triggering
    while the car is already inside the parking spot.
    """

    def __init__(
        self,
        controller,
        lock,
        cooldown: float = 30.0,
        min_confidence: float = 0.7,
        activation_distance: float = 3.0,
        approach_speed: int = 80,
        approach_time: float = 1.5,
        back_speed: int = 60,
        back_in_time: float = 2.0,
        align_time: float = 1.8,
        center_speed: int = 60,
        center_time: float = 0.5,
    ):
        """
        Args:
            controller: The SignController instance.
            lock: Threading lock shared with the controller.
            cooldown: Minimum seconds between activations.
            min_confidence: Minimum detection confidence to react.
            activation_distance: Maximum distance (m) at which the sign triggers.
            approach_speed: Forward speed (0–255 UI) during APPROACH phase.
            approach_time: Duration in seconds of the APPROACH phase.
            back_speed: Reverse speed (0–255 UI) during BACK_IN and ALIGN phases.
            back_in_time: Duration in seconds of the BACK_IN phase.
            align_time: Duration in seconds of the ALIGN phase.
            center_speed: Forward speed (0–255 UI) during CENTER phase.
            center_time: Duration in seconds of the CENTER phase.
        """
        super().__init__(controller, lock, min_confidence, activation_distance)
        self.cooldown = float(cooldown)
        self.last_activation_time: float = 0.0

        # Tunable timing / speed parameters
        self.approach_speed = int(max(0, min(255, approach_speed)))
        self.approach_time = float(approach_time)
        self.back_speed = int(max(0, min(255, back_speed)))
        self.back_in_time = float(back_in_time)
        self.align_time = float(align_time)
        self.center_speed = int(max(0, min(255, center_speed)))
        self.center_time = float(center_time)

        # State machine (protected by self.lock)
        self.phase: str = "IDLE"
        self.phase_start_time: float = 0.0
        self.is_running: bool = False
        self.is_parked: bool = False

        self._worker_thread: threading.Thread | None = None

    # ------------------------------------------------------------------
    # Public interface expected by SignController
    # ------------------------------------------------------------------

    def execute(self, detection: dict) -> bool:
        """
        Start the parking maneuver if conditions are met.

        This method is non-blocking: it launches a background thread that
        drives the phase sequence and returns immediately.

        Returns:
            True if the maneuver was started, False otherwise.
        """
        if not self.validate_detection(detection):
            return False

        now = time.time()

        if now - self.last_activation_time < self.cooldown:
            print("[ParkingStrategy] Cooldown active — ignoring parking sign.")
            return False

        with self.lock:
            if self.is_running:
                print("[ParkingStrategy] Maneuver already in progress — ignoring new trigger.")
                return False

            self.is_running = True
            self.is_parked = False
            self.phase = "APPROACH"
            self.phase_start_time = now

        label = detection["class"].lower()
        confidence = detection["confidence"]
        msg = (
            f"{label.upper()} DETECTED! ({confidence:.2f}) "
            "- Starting parallel parking maneuver"
        )
        print(f"[ParkingStrategy] {msg}")

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
            target=self._run_parking_sequence,
            name="ParkingStrategyWorker",
            daemon=True,
        )
        self._worker_thread.start()

        self.last_activation_time = now
        return True

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _set_motion(self, speed_ui: int, direction: str, servo_angle: int) -> None:
        """
        Send a combined speed + steering command and update controller state.

        Args:
            speed_ui: Speed value in UI scale (0–255).
            direction: "forward" or "backward".
            servo_angle: Servo position (SERVO_RIGHT=50, CENTER=105, SERVO_LEFT=160).
        """
        speed_ui = max(0, min(255, int(speed_ui)))
        direction = "backward" if direction == "backward" else "forward"

        ok_speed = self.controller.command_sender.send_speed_command(
            speed_ui, direction=direction
        )
        ok_steer = self.controller.command_sender.send_steering_command(servo_angle)

        with self.lock:
            if ok_speed:
                self.controller.current_speed = speed_ui
                self.controller.last_command = (
                    f"parking: speed={speed_ui} dir={direction} servo={servo_angle}"
                )
            if ok_speed and speed_ui > 0 and direction == "forward":
                self.controller.last_speed_before_stop = speed_ui

        if not ok_speed:
            print(
                f"[ParkingStrategy] Warning: failed to send speed command "
                f"(speed={speed_ui}, dir={direction})"
            )
        if not ok_steer:
            print(
                f"[ParkingStrategy] Warning: failed to send steering command "
                f"(servo={servo_angle})"
            )

    def _transition_to(self, phase: str) -> None:
        """
        Thread-safe phase transition; updates phase and resets the phase timer.
        """
        with self.lock:
            self.phase = phase
            self.phase_start_time = time.time()
        print(f"[ParkingStrategy] Phase → {phase}")

    def _run_parking_sequence(self) -> None:
        """
        Background worker that drives the parking phase sequence.
        Runs at ~20 Hz and transitions through phases based on elapsed time.
        """
        print("[ParkingStrategy] Worker thread started.")

        try:
            while True:
                with self.lock:
                    phase = self.phase
                    start_t = self.phase_start_time
                    running = self.is_running

                if not running:
                    break

                elapsed = time.time() - start_t

                if phase == "APPROACH":
                    self._set_motion(self.approach_speed, "forward", SERVO_CENTER)
                    if elapsed >= self.approach_time:
                        self._transition_to("BACK_IN")

                elif phase == "BACK_IN":
                    self._set_motion(self.back_speed, "backward", SERVO_LEFT)
                    if elapsed >= self.back_in_time:
                        self._transition_to("ALIGN")

                elif phase == "ALIGN":
                    self._set_motion(self.back_speed, "backward", SERVO_RIGHT)
                    if elapsed >= self.align_time:
                        self._transition_to("CENTER")

                elif phase == "CENTER":
                    self._set_motion(self.center_speed, "forward", SERVO_CENTER)
                    if elapsed >= self.center_time:
                        self._transition_to("STOP")

                elif phase == "STOP":
                    self._set_motion(0, "forward", SERVO_CENTER)
                    with self.lock:
                        self.is_running = False
                        self.is_parked = True
                        self.phase = "IDLE"

                    print("[ParkingStrategy] Maneuver complete. is_parked=True")

                    if self.controller.event_callback:
                        self.controller.event_callback("parking_completed", {"is_parked": True})
                    break

                else:
                    print(f"[ParkingStrategy] Unknown phase '{phase}' — aborting safely.")
                    self._set_motion(0, "forward", SERVO_CENTER)
                    with self.lock:
                        self.is_running = False
                        self.phase = "IDLE"
                    break

                time.sleep(0.05)  # ~20 Hz

        except Exception as exc:
            print(f"[ParkingStrategy] Unexpected error in worker thread: {exc}")
            try:
                self._set_motion(0, "forward", SERVO_CENTER)
            except Exception:
                pass
            with self.lock:
                self.is_running = False
                self.phase = "IDLE"
