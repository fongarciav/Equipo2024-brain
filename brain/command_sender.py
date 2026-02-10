"""
Command Sender Module - Handles sending steering commands via UART.

Following SRP: This module only handles command sending.
"""


SERVO_CENTER = 105
SERVO_RIGHT = 50
ANGLE_MAX = 30
STEER_TENTHS_MIN = -250
STEER_TENTHS_MAX = 250
SPEED_MM_S_MIN = -500
SPEED_MM_S_MAX = 500
UART_TERMINATOR = ";;\r\n"


class CommandSender:
    """Handles sending commands via UART."""
    
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
        Send steering command.
        
        Args:
            servo_angle: Servo angle (50-160, where 105 is center)
            
        Returns:
            True if command was sent successfully, False otherwise
        """
        # Convert ESP32-style servo angle to STM32 steering tenths of degree.
        conversion_factor = (SERVO_CENTER - SERVO_RIGHT) / ANGLE_MAX
        steering_angle = (SERVO_CENTER - servo_angle) / conversion_factor
        steering_tenths = int(round(steering_angle * 10))
        steering_tenths = max(STEER_TENTHS_MIN, min(STEER_TENTHS_MAX, steering_tenths))

        command = f"#steer:{steering_tenths}{UART_TERMINATOR}"
        success, _message = self.write_uart_command(command)
        return success
    
    def send_speed_command(self, speed: int, direction: str = "forward") -> bool:
        """
        Send speed command.
        
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

        command = f"#speed:{scaled_speed}{UART_TERMINATOR}"
        success, _message = self.write_uart_command(command)
        return success

    def send_heartbeat(self) -> bool:
        """
        Send heartbeat signal.
        
        Returns:
            True if command was sent successfully, False otherwise
        """
        command = f"#alive:1{UART_TERMINATOR}"
        success, _message = self.write_uart_command(command)
        return success
