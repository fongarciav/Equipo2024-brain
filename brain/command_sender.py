"""
Command Sender Module - Sends commands to Nucleo (STM32) via UART.

Protocol matches Nucleo main.cpp g_serialMonitorSubscribers: #key:payload;;\r\n
Keys must be lowercase as registered: speed, steer, brake, alive, kl.
"""


SERVO_CENTER = 105
SERVO_RIGHT = 50
ANGLE_MAX = 30
STEER_TENTHS_MIN = -250
STEER_TENTHS_MAX = 250
SPEED_MM_S_MIN = -500
SPEED_MM_S_MAX = 500
UART_TERMINATOR = ";;\r\n"

# Nucleo serial keys (from main.cpp g_serialMonitorSubscribers - lowercase)
KEY_STEER = "steer"
KEY_SPEED = "speed"
KEY_BRAKE = "brake"
KEY_ALIVE = "alive"
KEY_KL = "kl"


class CommandSender:
    """Sends steering, speed, brake and heartbeat to Nucleo over UART."""

    def __init__(self, write_uart_command_func):
        """
        Initialize the command sender.

        Args:
            write_uart_command_func: Function that takes a UART command string
                                     and returns (success: bool, message: str)
        """
        self.write_uart_command = write_uart_command_func

    def send_steering_command(self, servo_angle: int) -> bool:
        """
        Send steering command (Nucleo: serialCallbackSTEERcommand).

        Args:
            servo_angle: Servo angle (50-160, where 105 is center)

        Returns:
            True if command was sent successfully, False otherwise
        """
        conversion_factor = (SERVO_CENTER - SERVO_RIGHT) / ANGLE_MAX
        steering_angle = (SERVO_CENTER - servo_angle) / conversion_factor
        steering_tenths = int(round(steering_angle * 10))
        steering_tenths = max(STEER_TENTHS_MIN, min(STEER_TENTHS_MAX, steering_tenths))

        command = f"#{KEY_STEER}:{steering_tenths}{UART_TERMINATOR}"
        success, _message = self.write_uart_command(command)
        return success

    def send_speed_command(self, speed: int, direction: str = "forward") -> bool:
        """
        Send speed command (Nucleo: serialCallbackSPEEDcommand).

        Args:
            speed: Speed value (0-255)
            direction: Direction ("forward" or "backward")

        Returns:
            True if command was sent successfully, False otherwise
        """
        scaled_speed = int(round((speed / 255) * SPEED_MM_S_MAX))
        if direction.lower() == "backward":
            scaled_speed = -abs(scaled_speed)
        else:
            scaled_speed = abs(scaled_speed)
        scaled_speed = max(SPEED_MM_S_MIN, min(SPEED_MM_S_MAX, scaled_speed))

        command = f"#{KEY_SPEED}:{scaled_speed}{UART_TERMINATOR}"
        success, _message = self.write_uart_command(command)
        return success

    def send_brake(self) -> bool:
        """
        Send brake command (Nucleo: serialCallbackBRAKEcommand).

        Returns:
            True if command was sent successfully, False otherwise
        """
        command = f"#{KEY_BRAKE}:0{UART_TERMINATOR}"
        success, _message = self.write_uart_command(command)
        return success

    def send_heartbeat(self) -> bool:
        """
        Send heartbeat (Nucleo: serialCallbackAlivecommand).

        Returns:
            True if command was sent successfully, False otherwise
        """
        command = f"#{KEY_ALIVE}:1{UART_TERMINATOR}"
        success, _message = self.write_uart_command(command)
        return success
