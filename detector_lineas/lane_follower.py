#!/usr/bin/env python3
"""
Lane Follower - Integrates lane detection with UART control

This script connects the lane detection module with the UART controller,
following the Single Responsibility Principle and Composition over Inheritance.
"""

import sys
import argparse
import time
import signal

from deteccion_carril import run_lane_detection, choose_camera_by_OS
from uart_controller import UARTController, select_port_interactive


class LaneFollowerController:
    """
    Controller that connects lane detection events with UART commands.
    
    This class handles the composition of lane detection and UART control,
    following the Single Responsibility Principle.
    """
    
    def __init__(self, uart_controller: UARTController, speed: int = 180):
        """
        Initialize the lane follower controller.
        
        Args:
            uart_controller: UART controller instance
            speed: Speed value to maintain (180-255)
        """
        self.uart_controller = uart_controller
        self.speed = max(180, min(255, speed))
        self.last_speed_sent = 0
    
    def on_steering_detected(self, steering_angle: float):
        """
        Callback function called when a steering angle is detected.
        
        Args:
            steering_angle: Steering angle in degrees (-22 to +22)
        """
        # Send steering command
        self.uart_controller.set_steering(steering_angle)
        
        # Send speed command periodically (every 10 calls to reduce UART load)
        self.last_speed_sent += 1
        if self.last_speed_sent >= 10:
            self.uart_controller.set_speed(self.speed)
            self.last_speed_sent = 0


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Lane following with UART control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Interactive port selection
  python lane_follower.py --uart-interactive --arm-system
  
  # Specific port
  python lane_follower.py --uart-port /dev/ttyUSB0 --arm-system --speed 200
  
  # Custom camera
  python lane_follower.py --uart-port COM5 --camera 0 --mode auto
        """
    )
    
    parser.add_argument("--uart-port", type=str, help="Serial port for UART control (e.g., /dev/ttyUSB0, COM5)")
    parser.add_argument("--uart-baud", type=int, default=115200, help="Baud rate for UART (default: 115200)")
    parser.add_argument("--uart-interactive", action="store_true", help="Select UART port interactively")
    parser.add_argument("--speed", type=int, default=180, help="Speed value (180-255). Minimum working speed is 180 (default: 180)")
    parser.add_argument("--camera", type=int, default=None, help="Camera index (default: auto-select via config)")
    parser.add_argument("--arm-system", action="store_true", help="Arm the system before starting")
    parser.add_argument("--mode", choices=["manual", "auto"], default="manual", help="System mode: manual or auto (default: manual)")
    parser.add_argument("--no-display", action="store_true", help="Disable display windows")
    parser.add_argument("--web-stream", action="store_true", help="Enable web streaming (accessible via browser)")
    parser.add_argument("--web-port", type=int, default=5000, help="Port for web streaming (default: 5000)")
    parser.add_argument("--signal-detection", action="store_true", help="Enable traffic signal detection (YOLO)")
    parser.add_argument("--signal-model", type=str, default="../detector_señales/weights/best.engine", help="Path to model file (.engine/.trt for TensorRT, .pt for YOLO)")
    parser.add_argument("--signal-conf", type=float, default=0.5, help="Confidence threshold for signal detection (default: 0.5)")
    
    args = parser.parse_args()
    
    # Validate speed
    MIN_SPEED = 180
    if args.speed < MIN_SPEED:
        print(f"Warning: Speed {args.speed} is below minimum working speed ({MIN_SPEED}). Setting to {MIN_SPEED}.")
        args.speed = MIN_SPEED
    if args.speed > 255:
        print(f"Warning: Speed {args.speed} exceeds maximum (255). Setting to 255.")
        args.speed = 255
    
    # Select UART port
    if args.uart_interactive:
        port = select_port_interactive()
    elif args.uart_port:
        port = args.uart_port
    else:
        print("Error: Must specify either --uart-port or --uart-interactive")
        sys.exit(1)
    
    # Initialize UART controller
    uart_controller = UARTController(port, args.uart_baud)
    
    try:
        # Connect to UART
        print(f"Opening UART port: {port} at {args.uart_baud} baud...")
        if not uart_controller.connect():
            print("Failed to connect to UART port")
            sys.exit(1)
        print("UART connection established!")
        
        # Arm system and set mode if requested
        if args.arm_system:
            print(f"Setting system mode to {args.mode}...")
            uart_controller.set_mode(args.mode)
            time.sleep(0.1)
            
            print("Arming system...")
            uart_controller.arm()
            time.sleep(0.2)
            print("System armed!")
        
        # Select camera
        if args.camera is not None:
            camera_path = args.camera
        else:
            camera_path = choose_camera_by_OS()
        
        # Create lane follower controller
        controller = LaneFollowerController(uart_controller, args.speed)
        
        # Inicializar web streamer si está habilitado
        web_streamer = None
        if args.web_stream:
            try:
                from web_streamer import WebStreamer
                web_streamer = WebStreamer(port=args.web_port)
                web_streamer.start()
            except ImportError:
                print("Error: Flask not installed. Install with: pip install flask")
                print("Web streaming disabled.")
                args.web_stream = False
        
        # Inicializar detector de señales si está habilitado
        signal_detector = None
        if args.signal_detection:
            try:
                from signal_detector import SignalDetector
                import os
                model_path = os.path.abspath(args.signal_model)
                if not os.path.exists(model_path):
                    print(f"Warning: Signal model not found at {model_path}")
                    print("Signal detection disabled.")
                else:
                    signal_detector = SignalDetector(model_path, conf_threshold=args.signal_conf)
                    if signal_detector.is_available():
                        print(f"Signal detection: ENABLED (model: {model_path})")
                    else:
                        print("Signal detection: DISABLED (model failed to load)")
                        signal_detector = None
            except ImportError as e:
                print(f"Error: Signal detector dependencies not installed: {e}")
                print("Install with: pip install ultralytics")
                print("Signal detection disabled.")
        
        # Setup signal handler for graceful shutdown
        def signal_handler(sig, frame):
            print("\n\nShutting down...")
            uart_controller.emergency_stop()
            time.sleep(0.1)
            uart_controller.disconnect()
            if web_streamer:
                web_streamer.stop()
            sys.exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        print(f"Starting lane following...")
        print(f"UART control: ENABLED (speed: {args.speed})")
        if args.web_stream:
            print(f"Web streaming: ENABLED (port: {args.web_port})")
        print("Press 'q' to quit or Ctrl+C to stop")
        
        # Run lane detection with callback
        run_lane_detection(
            camera_path=camera_path,
            steering_callback=controller.on_steering_detected,
            show_display=not args.no_display,
            web_streamer=web_streamer,
            signal_detector=signal_detector
        )
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        print("Stopping...")
        uart_controller.emergency_stop()
        time.sleep(0.1)
        uart_controller.disconnect()
        print("UART connection closed")


if __name__ == "__main__":
    main()

