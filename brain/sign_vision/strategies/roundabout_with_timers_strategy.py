"""
RoundaboutWithTimers: desactiva auto, hace ~20 cm a la derecha (10 cm girando derecha + 10 cm girando izquierda), reactiva auto.
Tiempos calculados por velocidad (distancia = velocidad × tiempo).
"""

import threading
import time

from .base_strategy import SignStrategy

# Same as command_sender / parking_strategy
SERVO_CENTER = 105
SERVO_LEFT = 160
SERVO_RIGHT = 50
# Velocidad en mm/s cuando speed_ui=255
SPEED_MM_S_MAX = 500


def _time_for_distance_mm(distance_mm: float, speed_ui: int) -> float:
    """Tiempo en segundos para recorrer distance_mm a speed_ui (0-255)."""
    if speed_ui <= 0:
        return 0.0
    speed_mm_s = (speed_ui / 255.0) * SPEED_MM_S_MAX
    return distance_mm / speed_mm_s


class RoundaboutWithTimersStrategy(SignStrategy):
    """
    Estrategia para mini-roundabout:
    1) Desactiva modo auto
    2) Girar ruedas un poco a la derecha, avanzar 10 cm (tiempo según velocidad)
    3) Girar un poco a la izquierda, avanzar 10 cm
    4) Reactivar modo auto
    """

    def __init__(
        self,
        controller,
        lock,
        cooldown: float = 15.0,
        min_confidence: float = 0.7,
        activation_distance: float = 3.0,
        # Distancia por tramo (mm)
        segment_mm: float = 100.0,
        # Ángulo servo "un poco" derecha/izquierda (105 = centro; 50 = full right; 160 = full left)
        servo_right_bit: int = 85,
        servo_left_bit: int = 125,
        # Velocidad (0-255) usada en el avance si current_speed es 0
        default_maneuver_speed: int = 80,
    ):
        super().__init__(controller, lock, min_confidence, activation_distance)
        self.cooldown = float(cooldown)
        self.last_activation_time: float = 0.0
        self.segment_mm = float(segment_mm)
        self.servo_right_bit = max(SERVO_RIGHT, min(SERVO_CENTER, servo_right_bit))
        self.servo_left_bit = min(SERVO_LEFT, max(SERVO_CENTER, servo_left_bit))
        self.default_maneuver_speed = max(1, min(255, default_maneuver_speed))

        self._running = False
        self._worker: threading.Thread | None = None

    def execute(self, detection: dict) -> bool:
        if not self.validate_detection(detection):
            return False
        now = time.time()
        if now - self.last_activation_time < self.cooldown:
            print("[RoundaboutWithTimers] Cooldown activo, ignorando.")
            return False
        with self.lock:
            if self._running:
                print("[RoundaboutWithTimers] Ya en ejecución, ignorando.")
                return False
            self._running = True

        label = detection["class"].lower()
        confidence = detection["confidence"]
        msg = (
            f"{label.upper()} DETECTED! ({confidence:.2f}) "
            "- RoundaboutWithTimers: desactivando auto, 10cm derecha + 10cm izquierda, reactivar auto"
        )
        print(f"[RoundaboutWithTimers] {msg}")
        if self.controller.event_callback:
            self.controller.event_callback("sign_detected", {
                "label": label,
                "confidence": float(confidence),
                "message": msg,
            })

        self._worker = threading.Thread(
            target=self._run_sequence,
            name="RoundaboutWithTimersWorker",
            daemon=True,
        )
        self._worker.start()
        self.last_activation_time = now
        return True

    def _set_motion(self, speed_ui: int, servo_angle: int) -> None:
        speed_ui = max(0, min(255, int(speed_ui)))
        ok_speed = self.controller.command_sender.send_speed_command(speed_ui, direction="forward")
        ok_steer = self.controller.command_sender.send_steering_command(servo_angle)
        with self.lock:
            if ok_speed:
                self.controller.current_speed = speed_ui
                self.controller.last_command = (
                    f"roundabout: speed={speed_ui} servo={servo_angle}"
                )
        if not ok_speed:
            print(f"[RoundaboutWithTimers] Warning: no se envió speed (speed={speed_ui})")
        if not ok_steer:
            print(f"[RoundaboutWithTimers] Warning: no se envió steer (servo={servo_angle})")

    def _run_sequence(self) -> None:
        try:
            # 1) Desactivar modo auto
            ap = getattr(self.controller, "autopilot_controller", None)
            if ap is not None and hasattr(ap, "pause"):
                ap.pause()
                print("[RoundaboutWithTimers] Autopilot pausado.")

            with self.lock:
                speed_ui = self.controller.current_speed
            if speed_ui <= 0:
                speed_ui = self.default_maneuver_speed

            duration_s = _time_for_distance_mm(self.segment_mm, speed_ui)
            if duration_s <= 0:
                duration_s = _time_for_distance_mm(self.segment_mm, self.default_maneuver_speed)

            # 2) Girar un poco a la derecha y avanzar 10 cm
            self._set_motion(speed_ui, self.servo_right_bit)
            time.sleep(duration_s)

            # 3) Girar un poco a la izquierda y avanzar 10 cm
            self._set_motion(speed_ui, self.servo_left_bit)
            time.sleep(duration_s)

            # Parar y centrar
            self._set_motion(0, SERVO_CENTER)

            # 4) Reactivar modo auto
            if ap is not None and hasattr(ap, "resume"):
                ap.resume()
                print("[RoundaboutWithTimers] Autopilot reanudado.")
        except Exception as e:
            print(f"[RoundaboutWithTimers] Error en secuencia: {e}")
            ap = getattr(self.controller, "autopilot_controller", None)
            if ap is not None and hasattr(ap, "resume"):
                ap.resume()
            self._set_motion(0, SERVO_CENTER)
        finally:
            with self.lock:
                self._running = False
