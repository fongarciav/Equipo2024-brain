#!/usr/bin/env python3
"""
UART Controller Module

Handles all UART communication with the vehicle control system.
Follows Single Responsibility Principle - only handles UART communication.
"""

import sys
import time
from typing import Optional, Callable

try:
    import serial
    from serial.tools import list_ports
    UART_AVAILABLE = True
except ImportError:
    UART_AVAILABLE = False


def degrees_to_servo(degrees: float, max_degrees: float = 22.0) -> int:
    """
    Convierte grados de lane detector a valor de servo.
    
    Args:
        degrees: Ángulo en grados (-max_degrees a +max_degrees)
        max_degrees: Máximo ángulo permitido (default: 22°)
    
    Returns:
        Valor de servo (50-135)
    """
    SERVO_CENTER = 105
    SERVO_RANGE = 85  # 135 - 50
    
    # Normalizar a -1.0 a +1.0
    normalized = degrees / max_degrees
    # Limitar al rango [-1, 1]
    normalized = max(-1.0, min(1.0, normalized))
    # Convertir a valor de servo
    # NOTA: La convención está invertida - cuando degrees > 0 (derecha en detector),
    # el servo necesita un valor menor a 105 (izquierda en servo) para que el carro vaya a la derecha
    # Por lo tanto invertimos el signo
    servo_value = SERVO_CENTER - (normalized * (SERVO_RANGE / 2))
    return int(round(servo_value))


def select_port_interactive() -> str:
    """Select serial port interactively."""
    if not UART_AVAILABLE:
        raise ImportError("pyserial not available")
    
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found. Connect the ESP32 and try again.")
        sys.exit(1)
    if len(ports) == 1:
        return ports[0].device
    print("Available serial ports:")
    for idx, p in enumerate(ports):
        print(f"  [{idx}] {p.device} - {p.description}")
    while True:
        sel = input("Select port index: ").strip()
        if sel.isdigit() and 0 <= int(sel) < len(ports):
            return ports[int(sel)].device
        print("Invalid selection. Try again.")


class UARTController:
    """
    Controller for UART communication with vehicle.
    
    Handles:
    - Opening/closing serial port
    - Sending steering and speed commands
    - System arming/disarming
    - Emergency stop
    """
    
    def __init__(self, port: str, baud: int = 115200):
        """
        Initialize UART controller.
        
        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0', 'COM5')
            baud: Baud rate (default: 115200)
        """
        if not UART_AVAILABLE:
            raise ImportError("pyserial not available. Install with: pip install pyserial")
        
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self.is_connected = False
    
    def connect(self) -> bool:
        """Open serial port connection."""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=1.0,
                write_timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            # Clear any pending data
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.is_connected = True
            return True
        except Exception as e:
            print(f"Error opening UART port: {e}", file=sys.stderr)
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Close serial port connection."""
        if self.ser and self.ser.is_open:
            try:
                # Send emergency stop before closing
                self.emergency_stop()
                time.sleep(0.1)
            except:
                pass
            self.ser.close()
        self.is_connected = False
    
    def _write_line(self, msg: str) -> bool:
        """Write a line to serial port with error handling."""
        if not self.is_connected or not self.ser or not self.ser.is_open:
            return False
        
        line = (msg.strip() + "\n").encode("utf-8")
        try:
            self.ser.write(line)
            self.ser.flush()
            return True
        except serial.SerialTimeoutException:
            print("Warning: Write timeout - command may not have been sent", file=sys.stderr)
            return False
        except serial.SerialException as e:
            print(f"Error writing to serial: {e}", file=sys.stderr)
            return False
    
    def set_mode(self, mode: str) -> bool:
        """
        Set system mode.
        
        Args:
            mode: 'manual' or 'auto'
        
        Returns:
            True if command sent successfully
        """
        mode_value = 0 if mode == "manual" else 1
        return self._write_line(f"M:SYS_MODE:{mode_value}")
    
    def arm(self) -> bool:
        """
        Arm the system.
        
        Returns:
            True if command sent successfully
        """
        return self._write_line("M:SYS_ARM:0")
    
    def disarm(self) -> bool:
        """
        Disarm the system.
        
        Returns:
            True if command sent successfully
        """
        return self._write_line("M:SYS_DISARM:0")
    
    def set_steering(self, steering_angle_degrees: float) -> bool:
        """
        Set steering angle.
        
        Args:
            steering_angle_degrees: Steering angle in degrees (-22 to +22)
        
        Returns:
            True if command sent successfully
        """
        servo_value = degrees_to_servo(steering_angle_degrees, max_degrees=22.0)
        return self._write_line(f"C:SET_STEER:{servo_value}")
    
    def set_speed(self, speed: int) -> bool:
        """
        Set vehicle speed.
        
        Args:
            speed: Speed value (180-255, minimum working speed is 180)
        
        Returns:
            True if command sent successfully
        """
        # Validate speed
        speed = max(180, min(255, speed))
        return self._write_line(f"C:SET_SPEED:{speed}")
    
    def emergency_stop(self) -> bool:
        """
        Send emergency brake command.
        
        Returns:
            True if command sent successfully
        """
        return self._write_line("E:BRAKE_NOW:0")
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()

