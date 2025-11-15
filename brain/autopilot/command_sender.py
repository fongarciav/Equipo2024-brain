"""
Command Sender Module - Handles sending steering commands to ESP32 via UART.

Following SRP: This module only handles command sending.
"""


class CommandSender:
    """Handles sending commands to ESP32 via UART."""
    
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
        Send steering command to ESP32.
        
        Args:
            servo_angle: Servo angle (50-135, where 105 is center)
            
        Returns:
            True if command was sent successfully, False otherwise
        """
        # Format: C:SET_STEER:<angle>
        command = f"C:SET_STEER:{servo_angle}"
        success, message = self.write_uart_command(command)
        return success
    
    def send_speed_command(self, speed: int, direction: str = "forward") -> bool:
        """
        Send speed command to ESP32.
        
        Args:
            speed: Speed value (0-255)
            direction: Direction ("forward" or "backward")
            
        Returns:
            True if command was sent successfully, False otherwise
        """
        # Format: C:SET_SPEED:<speed>
        command = f"C:SET_SPEED:{speed}"
        success, message = self.write_uart_command(command)
        return success

