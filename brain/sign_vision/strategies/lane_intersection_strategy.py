"""
Estrategia de intersección detectada por lane (vista cenital).

Máquina de estado: solo la primera vez que se detecta una intersección,
después de 3 s se ejecuta turn_to_left_function() (doble a la izquierda).
En intersecciones siguientes no hace nada.
"""

import threading
import time

from .base_strategy import SignStrategy


class LaneIntersectionStrategy(SignStrategy):
    """
    Estrategia para intersección detectada por el lane detector.

    - turn_to_left: True → la primera vez que se ejecuta, tras 3 s se llama
      turn_to_left_function() y luego se pone a False.
    - turn_to_left: False → no hace nada (ya se hizo el giro en una intersección anterior).
    """

    def __init__(
        self,
        controller,
        lock,
        min_confidence: float = 0.5,
        activation_distance: float = 2.0,
        delay_seconds: float = 2,
    ):
        super().__init__(controller, lock, min_confidence, activation_distance)
        # Solo la primera intersección: hacer giro a la izquierda; después no.
        self.turn_to_left: bool = True
        self.delay_seconds = float(delay_seconds)

    def turn_to_left_function(self) -> None:
        """
        Doble a la izquierda. Por ahora no hace nada (stub).
        Se ejecuta una sola vez, 3 s después de detectar la primera intersección.
        """
        print("[LaneIntersectionStrategy] turn_to_left_function() ejecutado (doble a la izquierda)")

    def execute(self, detection: dict) -> bool:
        with self.lock:
            if not self.turn_to_left:
                return False

        # Primera vez: después de delay_seconds ejecutar giro a la izquierda
        def delayed_turn() -> None:
            time.sleep(self.delay_seconds)
            self.turn_to_left_function()
            with self.lock:
                self.turn_to_left = False

        threading.Thread(target=delayed_turn, daemon=True, name="LaneIntersectionDelayedTurn").start()
        return True
