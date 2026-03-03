import threading
import time

from .base_strategy import SignStrategy

# Servo angle constants (must match command_sender.py / angle_converter.py)
_SERVO_CENTER = 105
_SERVO_LEFT = 160
_SERVO_RIGHT = 50


class ParkingStrategy(SignStrategy):
    """
    Strategy for parallel parking to the left using a timed phase sequence.

    The sequence was updated to include an initial delay and a multi‑step
    backing maneuver to match the new requirements.  After the sign is
    detected the strategy will continue to let the line‑following code run
    while counting down a wait period proportional to the current speed.
    Once the wait expires the car executes the following timed motions:

        WAIT          → Countdown while leaving control to the line detector;
                        duration is calculated from current speed so the car
                        travels ~wait_target_distance_cm before maneuvering.
        BRAKE1        → Stop for 0.5 s before beginning the parking moves.
        BACK_RIGHT    → Reverse for 8 s with full‑right steering.
        BACK_LEFT     → Continue reversing for 4.5 s with full‑left steering.
        BRAKE2        → Stop for 0.5 s then prepare for a forward shift.
        FORWARD_RIGHT → Move forward for 4 s with full‑right steering.
        BRAKE3        → Stop (0.5 s) and center the wheels.
        BACK_CENTER   → Reverse with wheels centered for 2.5 s.
        STOP          → Hold speed=0 for stop_hold_duration s, then mark complete.

    A 30‑second cooldown still prevents re-triggering while the car is
    executing the maneuver.
    """
    def __init__(
        self,
        controller,
        lock,
        cooldown: float = 30.0,
        min_confidence: float = 0.6,
        activation_distance: float = 3.0,
        # speeds used during the various motion segments
        forward_approach_speed: int = 60,
        back_left_speed: int = 60,
        back_right_speed: int = 60,
        forward_center_speed: int = 0,
        # wait phase: duration is computed dynamically from current speed
        wait_target_distance_cm: float = 190.0,
        ui_speed_to_cm_per_s: float = 0.3,
        default_wait_duration: float = 8.0,
        wait_crawl_speed: int = 20,
        # configurable durations for the motion phases
        brake_duration: float = 0.5,
        back_right_duration1: float = 8.0,
        back_left_duration: float = 4.5,
        forward_right_duration: float = 4.0,
        back_center_duration: float = 2.5,
        stop_hold_duration: float = 1.0,
    ):
        """
        Args:
            controller: The SignController instance.
            lock: Threading lock shared with the controller.
            cooldown: Minimum seconds between activations.
            min_confidence: Minimum detection confidence to react.
            activation_distance: Maximum distance (m) at which the sign triggers.
            forward_approach_speed: Forward speed (0–255 UI) during FORWARD_RIGHT.
            back_left_speed: Reverse speed (0–255 UI) when steering full‑left.
            back_right_speed: Reverse speed (0–255 UI) when steering full‑right.
            forward_center_speed: Forward speed (0–255 UI) for centered forward motion
                (not used in current sequence, kept for compatibility).
            wait_target_distance_cm: Distance in cm the car should travel during
                the WAIT phase before starting the parking maneuver.
            ui_speed_to_cm_per_s: Conversion factor from UI speed units to cm/s.
                Default 1.0 (1:1). Calibrate empirically if needed.
            default_wait_duration: Fallback WAIT duration in seconds used when
                the car is stopped at the moment of detection.
            wait_crawl_speed: Speed (0–255 UI) sent to the car if it is stopped
                at detection time, so it advances to the maneuver position.
            brake_duration: How long to hold a full stop between motion phases.
            back_right_duration1: Time (s) to reverse with wheels full‑right.
            back_left_duration: Time (s) to reverse with wheels full‑left.
            forward_right_duration: Time (s) to drive forward with wheels full‑right.
            back_center_duration: Time (s) to reverse with wheels centered.
            stop_hold_duration: Time (s) to hold speed=0 in the final STOP phase
                before marking the maneuver complete, ensuring the stop command
                is received by the ESP32.
        """
        super().__init__(controller, lock, min_confidence, activation_distance)
        self.cooldown = float(cooldown)
        self.last_activation_time: float = 0.0

        # Tunable speeds for the various motion segments
        self.forward_approach_speed = int(max(0, min(255, forward_approach_speed)))
        self.back_left_speed = int(max(0, min(255, back_left_speed)))
        self.back_right_speed = int(max(0, min(255, back_right_speed)))
        self.forward_center_speed = int(max(0, min(255, forward_center_speed)))

        # Wait phase: dynamic duration parameters
        self.wait_target_distance_cm = float(wait_target_distance_cm)
        self.ui_speed_to_cm_per_s = float(ui_speed_to_cm_per_s)
        self.default_wait_duration = float(default_wait_duration)
        self.default_initial_speed = int(max(0, min(255, wait_crawl_speed)))

        # wait_duration is computed dynamically in execute(); initialised to default here
        self.wait_duration: float = self.default_wait_duration

        # Fixed durations used in the updated parking sequence
        self.brake_duration = float(brake_duration)
        self.back_right_duration1 = float(back_right_duration1)
        self.back_left_duration = float(back_left_duration)
        self.forward_right_duration = float(forward_right_duration)
        self.back_center_duration = float(back_center_duration)
        self.stop_hold_duration = float(stop_hold_duration)

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

            # Calculate wait_duration based on current speed so the car travels
            # approximately wait_target_distance_cm before starting the maneuver.
            current_speed_ui = self.controller.current_speed
            speed_cm_per_s = current_speed_ui * self.ui_speed_to_cm_per_s

            if speed_cm_per_s > 0:
                self.wait_duration = self.wait_target_distance_cm / speed_cm_per_s
            else:
                # Car is stopped: send a slow crawl speed so it reaches the maneuver
                # position, then compute wait_duration from that crawl speed.
                self.controller.command_sender.send_speed_command(self.default_initial_speed)
                self.wait_duration = self.wait_target_distance_cm / (self.default_initial_speed * self.ui_speed_to_cm_per_s)

            self.is_running = True
            self.is_parked = False
            # start in the wait phase so line following remains active
            self.phase = "WAIT"
            self.phase_start_time = now

        # Ensure autopilot is active during the wait so lane-following continues
        if hasattr(self.controller, "autopilot_controller") and self.controller.autopilot_controller:
            self.controller.autopilot_controller.resume()

        label = detection["class"].lower()
        confidence = detection["confidence"]
        msg = (
            f"{label.upper()} DETECTED! ({confidence:.2f}) "
            f"- Beginning parking countdown ({self.wait_duration:.1f}s)"
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

                if phase == "WAIT":
                    # No commands issued; lane-following (autopilot) remains in control.
                    if elapsed >= self.wait_duration:
                        if hasattr(self.controller, "autopilot_controller") and self.controller.autopilot_controller:
                            self.controller.autopilot_controller.pause()
                        self._transition_to("BRAKE1")

                elif phase == "BRAKE1":
                    self._set_motion(0, "forward", _SERVO_CENTER)
                    if elapsed >= self.brake_duration:
                        self._transition_to("BACK_RIGHT")

                elif phase == "BACK_RIGHT":
                    self._set_motion(self.back_right_speed, "backward", _SERVO_RIGHT)
                    if elapsed >= self.back_right_duration1:
                        self._transition_to("BACK_LEFT")

                elif phase == "BACK_LEFT":
                    self._set_motion(self.back_left_speed, "backward", _SERVO_LEFT)
                    if elapsed >= self.back_left_duration:
                        self._transition_to("BRAKE2")

                elif phase == "BRAKE2":
                    self._set_motion(0, "forward", _SERVO_CENTER)
                    if elapsed >= self.brake_duration:
                        self._transition_to("FORWARD_RIGHT")

                # FIX para que arranque
                elif phase == "FORWARD_RIGHT":
                    self._set_motion(self.forward_approach_speed, "forward", _SERVO_RIGHT)
                    if elapsed >= self.forward_right_duration:
                        self._transition_to("BRAKE3")

                elif phase == "BRAKE3":
                    self._set_motion(0, "forward", _SERVO_CENTER)
                    if elapsed >= self.brake_duration:
                        self._transition_to("BACK_CENTER")

                elif phase == "BACK_CENTER":
                    self._set_motion(self.back_left_speed, "backward", _SERVO_CENTER)
                    if elapsed >= self.back_center_duration:
                        self._transition_to("STOP")

                elif phase == "STOP":
                    # Hold speed=0 for stop_hold_duration seconds to ensure the ESP32
                    # receives and processes the stop command before the thread exits.
                    self._set_motion(0, "forward", _SERVO_CENTER)
                    if elapsed >= self.stop_hold_duration:
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
                    self._set_motion(0, "forward", _SERVO_CENTER)
                    with self.lock:
                        self.is_running = False
                        self.phase = "IDLE"
                    break

                time.sleep(0.05)  # ~20 Hz

        except Exception as exc:
            print(f"[ParkingStrategy] Unexpected error in worker thread: {exc}")
            try:
                self._set_motion(0, "forward", _SERVO_CENTER)
            except Exception:
                pass
            with self.lock:
                self.is_running = False
                self.phase = "IDLE"
