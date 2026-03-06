import json
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Optional

import serial


@dataclass
class UltraState:
    t: float = 0.0
    dist_cm: Optional[int] = None
    valid: bool = False
    present: bool = False
    threshold_cm: Optional[int] = None


class ThreadUltraUART(threading.Thread):
    """Reads NDJSON ultrasonic data from ESP32 over UART."""

    def __init__(
        self,
        port: str = "/dev/ttyTHS1",
        baud: int = 115200,
        stale_s: float = 0.25,
        name: str = "ThreadUltraUART",
    ):
        super().__init__(name=name, daemon=True)
        self.port = port
        self.baud = baud
        self.stale_s = stale_s
        self.state = UltraState()
        self._stop_event = threading.Event()
        self._ser = None
        self._lock = threading.Lock()

    def _is_port_busy(self) -> bool:
        """Best-effort check to avoid interfering with existing users."""
        try:
            result = subprocess.run(
                ["lsof", self.port],
                capture_output=True,
                text=True,
                timeout=1.0,
                check=False,
            )
            lines = [line for line in result.stdout.splitlines() if line.strip()]
            return len(lines) > 1
        except FileNotFoundError:
            # lsof not installed; skip check
            return False
        except Exception:
            return False

    def stop(self):
        self._stop_event.set()
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass

    def is_stale(self) -> bool:
        with self._lock:
            last = self.state.t
        return (time.time() - last) > self.stale_s

    def get_state_copy(self) -> UltraState:
        with self._lock:
            return UltraState(
                t=self.state.t,
                dist_cm=self.state.dist_cm,
                valid=self.state.valid,
                present=self.state.present,
                threshold_cm=self.state.threshold_cm,
            )

    def run(self):
        if self._is_port_busy():
            print(f"[UltraUART] Port {self.port} appears busy (lsof). Thread not started.")
            return

        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
        except Exception as e:
            print(f"[UltraUART] Cannot open {self.port}: {e}. Thread not started.")
            return

        print(f"[UltraUART] Reading {self.port} @ {self.baud}")

        while not self._stop_event.is_set():
            try:
                raw = self._ser.readline()
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                msg = json.loads(line)
                if msg.get("type") == "ultra" and msg.get("id") == "front":
                    with self._lock:
                        self.state.t = time.time()
                        self.state.dist_cm = msg.get("dist_cm")
                        self.state.valid = bool(msg.get("valid"))
                        self.state.present = bool(msg.get("present"))
                        self.state.threshold_cm = msg.get("threshold_cm")

            except json.JSONDecodeError:
                # Ignore non-JSON noise safely
                continue
            except Exception as e:
                print(f"[UltraUART] Read error on {self.port}: {e}. Thread terminating.")
                break

        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
